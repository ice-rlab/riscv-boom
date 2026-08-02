package boom.v3.perf

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.rocket.{
  PRV,
  IndirectCSRIO,
  SIndirectCSRRanges,
  MStatus,
  MARCtl,
  MARDepth,
  MARStatus
}

import boom.v3.common._

class MARData extends Bundle {
  val wpri0 = UInt(60.W)
  val tpe = UInt(4.W)
}

class MARRecord(implicit p: Parameters) extends BoomBundle {
  val pc = UInt(xLen.W)
  val addr = UInt(xLen.W)
  val time = UInt(xLen.W)
  val tpe = UInt(4.W)

  val isLoad = Bool()
  val isStore = Bool()

  val prv = UInt(2.W)
}

class MARLSUIO(implicit p: Parameters) extends BoomBundle {
  val records = Output(Vec(memWidth, Valid(new MARRecord())))
}

class MARIo(implicit p: Parameters) extends BoomBundle {
  // Indirect CSR entry-register access
  val scsrind = Flipped(
    new IndirectCSRIO(
      xLen,
      Seq(SIndirectCSRRanges.mar)
    )
  )

  // Direct MAR CSRs from CSR.scala
  val marctl = Input(new MARCtl())
  val smarstatus = Input(new MARStatus())
  val smardepth = Input(new MARDepth())
  //   val smarclr    = Input(Bool())

  // Hardware-updated status back to CSR.scala
  val smarstatus_next = Output(new MARStatus())
  val smarstatus_next_valid = Output(Bool())

  // Current time count from CSR/counter path
  val time = Input(UInt(xLen.W))

  // Interrupts
  val interrupt = Input(Bool())
  val interrupt_cause = Input(UInt(xLen.W))

  val status = Input(new MStatus())
  val lsu = Flipped(new MARLSUIO())
}

class MAR(implicit p: Parameters) extends BoomModule {
  val io = IO(new MARIo())

  private val marIdxWidth = log2Ceil(nMAREntries)

  private def marPhysIndex(writePtr: UInt, logicalIdx: UInt): UInt = {
    val offset = logicalIdx + 1.U
    (writePtr - offset)(marIdxWidth - 1, 0)
  }

  private def decodeDepth(depthEnc: UInt): UInt = {
    16.U << depthEnc
  }

  val writePtr = RegInit(0.U(marIdxWidth.W))

  val maraddress = RegInit(VecInit.fill(nMAREntries) {
    0.U(xLen.W)
  })

  val marpc = RegInit(VecInit.fill(nMAREntries) {
    0.U(xLen.W)
  })

  val martime = RegInit(VecInit.fill(nMAREntries) {
    0.U(xLen.W)
  })

  val mardata = RegInit(VecInit.fill(nMAREntries) {
    0.U.asTypeOf(new MARData())
  })

  io.smarstatus_next := io.smarstatus
  io.smarstatus_next_valid := false.B

  val isLcofiInterrupt =
    io.interrupt &&
      io.interrupt_cause(xLen - 1) &&
      io.interrupt_cause(log2Ceil(xLen) - 1, 0) === 13.U

  when(isLcofiInterrupt && io.marctl.lcofifrz) {
    io.smarstatus_next.frozen := true.B
    io.smarstatus_next_valid := true.B
  }

  /*
   * Write new MAR entries into the circular buffer.
   *
   * The kth valid new entry writes to:
   *   writePtr + k
   *
   * Then writePtr advances by nNew.
   */
  val marEnabled = !io.smarstatus.frozen

  val activeDepth = decodeDepth(io.smardepth.depth)

  val isNewMarEntry = Wire(Vec(memWidth, Bool()))

  for (i <- 0 until memWidth) {
    val rec = io.lsu.records(i).bits

    val prvAllowed =
      (rec.prv === PRV.U.U(2.W) && io.marctl.u) ||
        (rec.prv === PRV.S.U(2.W) && io.marctl.s) ||
        (rec.prv === PRV.M.U(2.W) && io.marctl.m)

    val loadAllowed =
      rec.isLoad && !io.marctl.loadinh

    val storeAllowed =
      rec.isStore && !io.marctl.storeinh

    isNewMarEntry(i) :=
      io.lsu.records(i).valid &&
        marEnabled &&
        prvAllowed &&
        (loadAllowed || storeAllowed)
  }

  val nNewMarEntries = PopCount(isNewMarEntry)

  for (i <- 0 until memWidth) {
    val priorNew = if (i == 0) 0.U else PopCount(isNewMarEntry.take(i))
    val wrIdx = (writePtr + priorNew)(marIdxWidth - 1, 0)

    val rec = io.lsu.records(i).bits

    when(isNewMarEntry(i)) {
      maraddress(wrIdx) := rec.addr
      marpc(wrIdx) := rec.pc
      martime(wrIdx) := rec.time
      mardata(wrIdx).tpe := rec.tpe
    }
  }

  when(nNewMarEntries =/= 0.U) {
    writePtr := writePtr + nNewMarEntries
  }
  // TODO: Clear instruction
  //   when(io.smarclr) {
  //     writePtr := 0.U

  //     for (i <- 0 until nMAREntries) {
  //       maraddress(i) := 0.U
  //       marpc(i)      := 0.U
  //       martime(i)   := 0.U
  //       mardata(i)    := 0.U.asTypeOf(new MARData())
  //     }

  //     io.smarstatus_next.wrptr := 0.U
  //     io.smarstatus_next_valid := true.B
  //   }

  /*
   * Indirect CSR read path.
   *
   * Logical MAR entries are exposed at indices:
   *   0x80000200 through 0x800002ff
   *
   * logical 0 = most recent entry
   * logical 1 = previous entry
   *
   * sireg  -> maraddress
   * sireg2 -> marpc
   * sireg3 -> martime
   * sireg4 -> mardata
   */
  val marRange = SIndirectCSRRanges.mar

  val inMarRange = marRange.hit(io.scsrind.index)
  val logicalIdx = marRange.offset(io.scsrind.index)
  val physIdx = marPhysIndex(writePtr, logicalIdx)

  val logicalInDepth = logicalIdx < activeDepth

  io.scsrind.resp(0).hit := inMarRange
  io.scsrind.resp(0).rdata := 0.U

  when(inMarRange && logicalInDepth) {
    when(io.scsrind.wen) {
      // MAR entries are read-only.
    }

    switch(io.scsrind.reg) {
      is(0.U) {
        io.scsrind.resp(0).rdata := maraddress(physIdx)
      }
      is(1.U) {
        io.scsrind.resp(0).rdata := marpc(physIdx)
      }
      is(2.U) {
        io.scsrind.resp(0).rdata := martime(physIdx)
      }
      is(3.U) {
        io.scsrind.resp(0).rdata := mardata(physIdx).asUInt
      }
    }
  }

  dontTouch(writePtr)
  dontTouch(maraddress)
  dontTouch(marpc)
  dontTouch(martime)
  dontTouch(mardata)
}
