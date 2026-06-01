package boom.v3.perf

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.util._
import freechips.rocketchip.rocket.{
  CTRCtl,
  CTRDepth,
  CTRStatus,
  IndirectCSRIO,
  IndirectCSRRange,
  MStatus
}

import boom.v3.ifu.{GetPCFromFtqIO}
import boom.v3.common._
import boom.v3.util._
import boom.v3.exu.{CommitSignals, BrResolutionInfo}

class CTRSource(implicit p: Parameters) extends BoomBundle {
  val pc = UInt((xLen - 1).W) // bits [XLEN-1:1]
  val v = Bool() // bit 0
}

class CTRTarget(implicit p: Parameters) extends BoomBundle {
  val pc = UInt((xLen - 1).W) // bits [XLEN-1:1]
  val misp = Bool() // bit 0
}

class CTRData extends Bundle {
  val wpri0 = UInt(32.W)
  val cc = UInt(16.W)
  val ccv = Bool()
  val wpri1 = UInt(11.W)
  val tpe = UInt(4.W)
}

class CTRIo(implicit p: Parameters) extends BoomBundle {
  val commit = Input(new CommitSignals())
  // TODO: We need to change this to get the PC from the FTQ, however this takes a cycle and can conflict with the other FTQ read port. Should the CTR sit at the branch unit?
  // val get_ftq_pc = new GetPCFromFtqIO()

  // Indirect CSR entry-register access
  val scsrind = Flipped(new IndirectCSRIO(xLen))

  // Direct CTR CSRs from CSR.scala
  val mctrctl = Input(new CTRCtl())
  val sctrstatus = Input(new CTRStatus())
  val sctrdepth = Input(new CTRDepth())
  val sctrclr = Input(new Bool())

  // Hardware-updated status back to CSR.scala
  val sctrstatus_next = Output(new CTRStatus())
  val sctrstatus_next_valid = Output(Bool())

  // Interrupts
  val interrupt = Input(new Bool())
  val interrupt_cause = Input(Output(UInt(xLen.W)))

  val status = Input(new MStatus())
}

class CTR(implicit p: Parameters) extends BoomModule {
  val io = IO(new CTRIo())

  require(nCTREntries > 0)
  require(isPow2(nCTREntries))

  private val ctrIdxWidth = log2Ceil(nCTREntries)

  private def ctrPhysIndex(writePtr: UInt, logicalIdx: UInt): UInt = {
    val offset = logicalIdx + 1.U
    (writePtr - offset)(ctrIdxWidth - 1, 0)
  }

  def ctrType(uop: MicroOp): UInt = {
    val enc = WireDefault(0.U(4.W))

    // Taken and not taken is wrong right now, its predicted and not real
    when(uop.exception) {
      enc := 1.U // Exception
    }.elsewhen(uop.is_br && !uop.taken) {
      enc := 4.U // Not-taken branch
    }.elsewhen(uop.is_br && uop.taken) {
      enc := 5.U // Taken branch
    }.elsewhen(uop.is_jalr && uop.ldst_val) {
      enc := 8.U // Indirect call / linked indirect jump
    }.elsewhen(uop.is_jal && uop.ldst_val) {
      enc := 9.U // Direct call
    }.elsewhen(uop.is_jalr && !uop.ldst_val) {
      enc := 10.U // Indirect jump without linkage
    }.elsewhen(uop.is_jal && !uop.ldst_val) {
      enc := 11.U // Direct jump without linkage
    }
    // What is trap return?
    // .elsewhen(uop.flush_on_commit) {
    //   enc := 3.U // Trap return / system-return-ish placeholder
    // }
    enc
  }

  val writePtr = RegInit(0.U(ctrIdxWidth.W))

  val ctrsource = RegInit(VecInit.fill(nCTREntries) {
    0.U.asTypeOf(new CTRSource())
  })

  val ctrtarget = RegInit(VecInit.fill(nCTREntries) {
    0.U.asTypeOf(new CTRTarget())
  })

  val ctrdata = RegInit(VecInit.fill(nCTREntries) {
    0.U.asTypeOf(new CTRData())
  })

  io.sctrstatus_next := io.sctrstatus
  io.sctrstatus_next_valid := false.B

  val isLcofiInterrupt =
    io.interrupt &&
      io.interrupt_cause(xLen - 1) &&
      io.interrupt_cause(log2Ceil(xLen) - 1, 0) === 13.U

  when(isLcofiInterrupt && io.mctrctl.lcofifrz) {
    io.sctrstatus_next.frozen := true.B
    io.sctrstatus_next_valid := true.B
  }

  /*
   * Build source -> target pairs
   */
  val isRetiring = VecInit((0 until retireWidth).map { i =>
    io.commit.valids(i)
  })

  val lastUop = RegInit(0.U.asTypeOf(new MicroOp()))

  when(isRetiring.asUInt =/= 0.U) {
    lastUop := PriorityMux(isRetiring, io.commit.uops)
  }

  val commitPairs: Seq[(MicroOp, Bool)] =
    io.commit.uops.zip(io.commit.valids)

  val uops: Seq[(MicroOp, Bool)] =
    (lastUop, true.B) +: commitPairs

  val isCfi: Seq[Bool] = uops
    .dropRight(1)
    .map { case (u, v) =>
      v && ((u.is_br && u.taken) || u.is_jal || u.is_jalr)
    }

  val isNewCtrEntry: Seq[Bool] =
    isCfi.zipWithIndex.map { case (cfi, i) =>
      cfi && uops(i + 1)._2
    }

  val ctrFrozen = io.sctrstatus.frozen
  // If we are fozen, don't accept any new entries
  val nNew = Mux(
    ctrFrozen,
    0.U,
    PopCount(isNewCtrEntry)
  )

  /*
   * Write new CTR entries into the circular buffer.
   *
   * The kth valid new entry writes to:
   *   writePtr + k
   *
   * Then writePtr advances by nNew.
   */
  for (i <- 0 until retireWidth) {
    val priorNew = if (i == 0) 0.U else PopCount(isNewCtrEntry.take(i))
    val wrIdx = (writePtr + priorNew)(ctrIdxWidth - 1, 0)

    val src = Wire(new CTRSource())
    val tgt = Wire(new CTRTarget())
    val dat = Wire(new CTRData())

    src := 0.U.asTypeOf(new CTRSource())
    tgt := 0.U.asTypeOf(new CTRTarget())
    dat := 0.U.asTypeOf(new CTRData())

    src.v := true.B

    // This is just for basic setup we debug_pc. However, this is not the correct approach.
    // Ideally, we should go over the ftq to reconstruct the PC. However there are some challenges here. For instance, how do we arbitrate the ftq read port compared to branch execution units.
    src.pc := uops(i)._1.debug_pc(vaddrBitsExtended - 1, 1)

    tgt.misp := false.B
    tgt.pc := uops(i + 1)._1.debug_pc(vaddrBitsExtended - 1, 1)

    dat.tpe := ctrType(uops(i)._1)

    when(isNewCtrEntry(i)) {
      ctrsource(wrIdx) := src
      ctrtarget(wrIdx) := tgt
      ctrdata(wrIdx) := dat
    }
  }

  when(io.sctrclr) {
    writePtr := 0.U

    for (i <- 0 until nCTREntries) {
      ctrsource(i) := 0.U.asTypeOf(new CTRSource())
      ctrtarget(i) := 0.U.asTypeOf(new CTRTarget())
      ctrdata(i) := 0.U.asTypeOf(new CTRData())
    }
  }

  when(nNew =/= 0.U) {
    writePtr := writePtr + nNew
  }

  /*
   * Indirect CSR read path.
   *
   * Logical CTR entries are exposed at indices:
   *   0x200 through 0x200 + nCTREntries - 1
   *
   * logical 0 = most recent entry
   * logical 1 = previous entry
   */
  val ctr = IndirectCSRRange(
    name = "ctr",
    base = 0x200,
    size = nCTREntries
  )

  val inCtrRange =
    io.scsrind.index >= ctr.base.U &&
      io.scsrind.index < (ctr.base + ctr.size).U

  val logicalIdx = io.scsrind.index - ctr.base.U
  val physIdx = ctrPhysIndex(writePtr, logicalIdx)

  io.scsrind.rdata := 0.U

  when(inCtrRange) {
    when(io.scsrind.wen) {
      // CTR entries are read-only for now.
    }

    switch(io.scsrind.reg) {
      is(0.U) {
        io.scsrind.rdata := ctrsource(physIdx).asUInt
      }
      is(1.U) {
        io.scsrind.rdata := ctrtarget(physIdx).asUInt
      }
      is(2.U) {
        io.scsrind.rdata := ctrdata(physIdx).asUInt
      }
    }
  }

  dontTouch(writePtr)
  dontTouch(ctrsource)
  dontTouch(ctrtarget)
  dontTouch(ctrdata)
}
