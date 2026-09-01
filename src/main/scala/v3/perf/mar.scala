package boom.v3.perf

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.rocket.{PRV, IndirectCSRIO, SIndirectCSRRanges, MStatus, MARCtl, MARDepth, MARStatus}

import boom.v3.exu.CommitSignals
import boom.v3.common._

class MARData extends Bundle {
  val wpri0 = UInt(58.W)
  val prv = UInt(2.W)
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
  val rob_idx = UInt(robAddrSz.W)
}

class MARStoredRecord(implicit p: Parameters) extends BoomBundle {
  val address = UInt(xLen.W)
  val pc = UInt(xLen.W)
  val time = UInt(xLen.W)
  val data = new MARData()
}

class MARLSUIO(implicit p: Parameters) extends BoomBundle {
  val records = Output(Vec(memWidth, Valid(new MARRecord())))
}

class MARIo(implicit p: Parameters) extends BoomBundle {
  val scsrind = Flipped(new IndirectCSRIO(xLen, Seq(SIndirectCSRRanges.mar)))

  val marctl = Input(new MARCtl())
  val smarstatus = Input(new MARStatus())
  val smardepth = Input(new MARDepth())

  val smaralb = Input(UInt(xLen.W))
  val smaraub = Input(UInt(xLen.W))

  val smarstatus_next = Output(new MARStatus())
  val smarstatus_next_valid = Output(Bool())

  val time = Input(UInt(xLen.W))

  val interrupt = Input(Bool())
  val interrupt_cause = Input(UInt(xLen.W))

  val status = Input(new MStatus())


  // decode input
  val enq_valids = Input(Vec(coreWidth, Bool()))
  val enq_uops = Input(Vec(coreWidth, new MicroOp()))

  // lsu signals
  val lsu = Flipped(new MARLSUIO())
  
  // commit signals
  val commit = Input(new CommitSignals())
}

class MAR(implicit p: Parameters) extends BoomModule {
  val io = IO(new MARIo())

  private val marIdxWidth = log2Ceil(nMAREntries)
  private val maxWritesPerCycle = retireWidth + memWidth
  private val nMARBanks = 1 << log2Ceil(maxWritesPerCycle)
  private val marBankBits = log2Ceil(nMARBanks)
  private val marBankDepth = nMAREntries / nMARBanks
  private val marRowWidth = math.max(1, log2Ceil(marBankDepth))

  require((nMAREntries & (nMAREntries - 1)) == 0, "MAR entry count must be a power of two")
  require(nMARBanks <= nMAREntries, "MAR requires at least as many entries as banks")
  require(nMAREntries % nMARBanks == 0, "MAR entry count must be divisible by the number of banks")

  private def marPhysIndex(writePtr: UInt, logicalIdx: UInt): UInt = {
    val offset = logicalIdx + 1.U
    (writePtr - offset)(marIdxWidth - 1, 0)
  }

  private def decodeDepth(depthEnc: UInt): UInt = {
    16.U << depthEnc
  }

  private def marBank(idx: UInt): UInt = {
    if (nMARBanks == 1) 0.U else idx(marBankBits - 1, 0)
  }

  private def marRow(idx: UInt): UInt = {
    if (marBankDepth == 1) 0.U(marRowWidth.W) else idx(marIdxWidth - 1, marBankBits)
  }

  private def assignStored(dst: MARStoredRecord, src: MARRecord): Unit = {
    dst.address := src.addr
    dst.pc := src.pc
    dst.time := src.time
    dst.data.wpri0 := 0.U
    dst.data.prv := src.prv
    dst.data.tpe := src.tpe
  }

  //--------------------------------------------------------------------------
  // CSR-visible MAR circular buffer
  //--------------------------------------------------------------------------

  val writePtr = RegInit(0.U(marIdxWidth.W))
  val validEntries = RegInit(0.U((marIdxWidth + 1).W))

  val marBanks = Seq.fill(nMARBanks) {
    Mem(marBankDepth, new MARStoredRecord())
  }

  //--------------------------------------------------------------------------
  // Pending speculative MAR entries
  //--------------------------------------------------------------------------

  val pendingAllocated = RegInit(VecInit.fill(numRobEntries)(false.B))
  val pendingMemoryValid = RegInit(VecInit.fill(numRobEntries)(false.B))
  val pendingRecord = Reg(Vec(numRobEntries, new MARRecord()))

  //--------------------------------------------------------------------------
  // MAR status / freeze
  //--------------------------------------------------------------------------

  io.smarstatus_next := io.smarstatus
  io.smarstatus_next_valid := false.B

  val isLcofiInterrupt =
    io.interrupt && io.interrupt_cause(xLen - 1) && io.interrupt_cause(log2Ceil(xLen) - 1, 0) === 13.U

  val freezeNow = isLcofiInterrupt && io.marctl.lcofifrz

  when(freezeNow) {
    io.smarstatus_next.frozen := true.B
    io.smarstatus_next_valid := true.B
  }

  val acceptNewMAR = !io.smarstatus.frozen
  val activeDepth = decodeDepth(io.smardepth.depth)

  //--------------------------------------------------------------------------
  // Address-range filter
  //
  // smaralb <= addr < smaraub
  //
  // 0 / 0 disables address filtering.
  //--------------------------------------------------------------------------

  val addressFilterEnabled = io.smaralb =/= 0.U || io.smaraub =/= 0.U

  //--------------------------------------------------------------------------
  // Capture the architectural cycle counter with each incoming LSU request
  //--------------------------------------------------------------------------

  val incomingRecords = Wire(Vec(memWidth, new MARRecord()))

  for (i <- 0 until memWidth) {
    incomingRecords(i) := io.lsu.records(i).bits
    incomingRecords(i).time := io.time
  }

  //--------------------------------------------------------------------------
  // Filter incoming LSU records
  //--------------------------------------------------------------------------

  val isNewMarEntry = Wire(Vec(memWidth, Bool()))
  val needsRetirement = Wire(Vec(memWidth, Bool()))
  val directVisible = Wire(Vec(memWidth, Bool()))

  for (i <- 0 until memWidth) {
    val rec = incomingRecords(i)

    val prvAllowed =
      (rec.prv === PRV.U.U(2.W) && io.marctl.u) ||
      (rec.prv === PRV.S.U(2.W) && io.marctl.s) ||
      (rec.prv === PRV.M.U(2.W) && io.marctl.m)

    val loadAllowed = rec.isLoad && !io.marctl.loadinh
    val storeAllowed = rec.isStore && !io.marctl.storeinh
    val isAmo = rec.tpe(2)
    val isHella = rec.tpe(3)

    val addressAllowed = !addressFilterEnabled || (rec.addr >= io.smaralb && rec.addr < io.smaraub)

    isNewMarEntry(i) :=
      io.lsu.records(i).valid && acceptNewMAR && !isHella && prvAllowed && addressAllowed && (loadAllowed || storeAllowed)

    needsRetirement(i) := rec.isLoad || isAmo
    directVisible(i) := isNewMarEntry(i) && rec.isStore && !isAmo
  }

  //--------------------------------------------------------------------------
  // Pending-buffer allocation at dispatch
  //--------------------------------------------------------------------------

  for (w <- 0 until coreWidth) {
    when(io.enq_valids(w)) {
      val robIdx = io.enq_uops(w).rob_idx

      pendingAllocated(robIdx) := true.B
      pendingMemoryValid(robIdx) := false.B
      pendingRecord(robIdx) := 0.U.asTypeOf(new MARRecord())
      pendingRecord(robIdx).pc := io.enq_uops(w).debug_pc
      pendingRecord(robIdx).rob_idx := robIdx
    }
  }

  //--------------------------------------------------------------------------
  // Pending-buffer validation and fill from the LSU
  //--------------------------------------------------------------------------

  for (i <- 0 until memWidth) {
    val rec = incomingRecords(i)
    val robIdx = rec.rob_idx

    when(isNewMarEntry(i) && needsRetirement(i) && pendingAllocated(robIdx)) {
      pendingMemoryValid(robIdx) := true.B
      pendingRecord(robIdx) := rec
    }
  }

  //--------------------------------------------------------------------------
  // ROB commit matching
  //--------------------------------------------------------------------------

  val retiredPendingValid = Wire(Vec(retireWidth, Bool()))
  val retiredPendingRecord = Wire(Vec(retireWidth, new MARRecord()))

  for (c <- 0 until retireWidth) {
    val commitUop = io.commit.uops(c)
    val robIdx = commitUop.rob_idx
    val newPendingMatch = Wire(Vec(memWidth, Bool()))

    for (i <- 0 until memWidth) {
      newPendingMatch(i) :=
        isNewMarEntry(i) && needsRetirement(i) && incomingRecords(i).rob_idx === robIdx
    }

    val hasNewPendingMatch = newPendingMatch.asUInt.orR
    val bypassRecord = Mux1H(newPendingMatch.toSeq, incomingRecords.toSeq)
    val candidateRecord = Mux(hasNewPendingMatch, bypassRecord, pendingRecord(robIdx))
    val candidateValid = pendingAllocated(robIdx) &&
      (hasNewPendingMatch || pendingMemoryValid(robIdx))

    val pcMatches = candidateRecord.pc === commitUop.debug_pc

    val typeMatches =
      (candidateRecord.isLoad && commitUop.uses_ldq) ||
      (candidateRecord.isStore && commitUop.uses_stq)

    retiredPendingValid(c) := io.commit.arch_valids(c) && candidateValid && pcMatches && typeMatches
    retiredPendingRecord(c) := candidateRecord

    when(io.commit.valids(c)) {
      pendingAllocated(robIdx) := false.B
      pendingMemoryValid(robIdx) := false.B
    }
  }

  //--------------------------------------------------------------------------
  // Build ordered MAR write candidates
  //
  // Order:
  //
  //   1. Retired speculative loads / AMOs
  //   2. Committed stores draining from the LSU
  //--------------------------------------------------------------------------

  val nRetiredPending = PopCount(retiredPendingValid)
  val nDirectVisible = PopCount(directVisible)
  val nVisibleEntries = nRetiredPending +& nDirectVisible

  val writeValid = Wire(Vec(maxWritesPerCycle, Bool()))
  val writeIndex = Wire(Vec(maxWritesPerCycle, UInt(marIdxWidth.W)))
  val writeRecord = Wire(Vec(maxWritesPerCycle, new MARStoredRecord()))

  for (w <- 0 until maxWritesPerCycle) {
    writeValid(w) := false.B
    writeIndex(w) := 0.U
    writeRecord(w) := 0.U.asTypeOf(new MARStoredRecord())
  }

  for (c <- 0 until retireWidth) {
    val priorRetired = if (c == 0) 0.U else PopCount(retiredPendingValid.take(c))
    val wrIdx = (writePtr + priorRetired)(marIdxWidth - 1, 0)

    writeValid(c) := retiredPendingValid(c)
    writeIndex(c) := wrIdx
    assignStored(writeRecord(c), retiredPendingRecord(c))
  }

  for (i <- 0 until memWidth) {
    val priorDirect = if (i == 0) 0.U else PopCount(directVisible.take(i))
    val wrIdx = (writePtr + nRetiredPending + priorDirect)(marIdxWidth - 1, 0)
    val slot = retireWidth + i

    writeValid(slot) := directVisible(i)
    writeIndex(slot) := wrIdx
    assignStored(writeRecord(slot), incomingRecords(i))
  }

  //--------------------------------------------------------------------------
  // Banked MAR writes
  //
  // The number of banks is the next power of two >= the maximum number of
  // writes per cycle. Since writes are consecutive, every valid write in a
  // cycle maps to a different bank.
  //--------------------------------------------------------------------------

  val writeBank = Wire(Vec(maxWritesPerCycle, UInt(math.max(1, marBankBits).W)))
  val writeRow = Wire(Vec(maxWritesPerCycle, UInt(marRowWidth.W)))

  for (w <- 0 until maxWritesPerCycle) {
    writeBank(w) := marBank(writeIndex(w))
    writeRow(w) := marRow(writeIndex(w))
  }

  for (b <- 0 until nMARBanks) {
    val matches = Wire(Vec(maxWritesPerCycle, Bool()))

    for (w <- 0 until maxWritesPerCycle) {
      matches(w) := writeValid(w) && writeBank(w) === b.U
    }

    val bankWriteValid = matches.asUInt.orR
    val bankWriteRow = Mux1H(matches.toSeq, writeRow.toSeq)
    val bankWriteRecord = Mux1H(matches.toSeq, writeRecord.toSeq)

    assert(PopCount(matches) <= 1.U)

    when(bankWriteValid) {
      marBanks(b).write(bankWriteRow, bankWriteRecord)
    }
  }

  when(nVisibleEntries =/= 0.U) {
    writePtr := writePtr + nVisibleEntries

    val nextValidEntries = validEntries +& nVisibleEntries

    validEntries := Mux(nextValidEntries >= nMAREntries.U, nMAREntries.U, nextValidEntries)
  }

  //--------------------------------------------------------------------------
  // TODO: Clear instruction
  //--------------------------------------------------------------------------

  // when(io.smarclr) {
  //   writePtr := 0.U
  //   validEntries := 0.U
  //
  //   for (i <- 0 until numRobEntries) {
  //     pendingAllocated(i) := false.B
  //     pendingMemoryValid(i) := false.B
  //   }
  //
  //   io.smarstatus_next.wrptr := 0.U
  //   io.smarstatus_next_valid := true.B
  // }

  //--------------------------------------------------------------------------
  // Indirect CSR read path
  //--------------------------------------------------------------------------

  val marRange = SIndirectCSRRanges.mar
  val inMarRange = marRange.hit(io.scsrind.index)
  val logicalIdx = marRange.offset(io.scsrind.index)
  val physIdx = marPhysIndex(writePtr, logicalIdx)

  val logicalInDepth = logicalIdx < activeDepth
  val logicalWritten = logicalIdx < validEntries

  val readBank = marBank(physIdx)
  val readRow = marRow(physIdx)

  val bankReadData = Wire(Vec(nMARBanks, new MARStoredRecord()))

  for (b <- 0 until nMARBanks) {
    bankReadData(b) := marBanks(b).read(readRow)
  }

  val selectedReadData =
    if (nMARBanks == 1) bankReadData(0)
    else Mux1H(UIntToOH(readBank, nMARBanks), bankReadData)

  io.scsrind.resp(0).hit := inMarRange
  io.scsrind.resp(0).rdata := 0.U

  when(inMarRange && logicalInDepth && logicalWritten) {
    switch(io.scsrind.reg) {
      is(0.U) {
        io.scsrind.resp(0).rdata := selectedReadData.address
      }

      is(1.U) {
        io.scsrind.resp(0).rdata := selectedReadData.pc
      }

      is(2.U) {
        io.scsrind.resp(0).rdata := selectedReadData.time
      }

      is(3.U) {
        io.scsrind.resp(0).rdata := selectedReadData.data.asUInt
      }
    }
  }

  dontTouch(io.smaralb)
  dontTouch(io.smaraub)
}