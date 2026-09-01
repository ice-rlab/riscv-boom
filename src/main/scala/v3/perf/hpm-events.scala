package boom.v3.perf

import chisel3._
import chisel3.util.{PopCount, log2Ceil}
import org.chipsalliance.cde.config.Parameters

import boom.v3.common._
import boom.v3.exu.{CommitSignals}
import freechips.rocketchip.rocket.{EventSet, EventSets}

class BoomPerfHPMEventsIO(implicit p: Parameters)
  extends BoomBundle()(p)
  with HasBoomCoreParameters
{
  // Branch and redirect events
  val branch_mispredict = Input(Bool())
  val cfi_target_mispredict = Input(Bool())
  val branch_resolved = Input(Bool())

  // ROB and commit events
  val rob_flush = Input(Bool())
  val commit_uops = Input(Vec(retireWidth, Bool()))

  // Memory-system events
  val ifu_acquire = Input(Bool())
  val lsu_acquire = Input(Bool())
  val lsu_release = Input(Bool())
  val ifu_tlb_miss = Input(Bool())
  val lsu_tlb_miss = Input(Bool())
  val ptw_l2_miss = Input(Bool())

  // Frontend and decode events
  val fetchpacket_valid = Input(Bool())
  val dec_valids = Input(Vec(coreWidth, Bool()))
  val dec_fbundle_valids = Input(Vec(coreWidth, Bool()))
  val dec_stalls = Input(Vec(coreWidth, Bool()))
  val dec_fire = Input(Vec(coreWidth, Bool()))

  // ROB commit signals
  val commit = Input(new CommitSignals())

  // Flush events
  val sfence_valid = Input(Bool())
  val redirect_val = Input(Bool())
  val redirect_flush = Input(Bool())

  // Retired instruction-type events
  val jump_retired = Output(UInt(retireWidth.W))
  val branch_retired = Output(UInt(retireWidth.W))
  val memory_retired = Output(UInt(retireWidth.W))
  val load_retired = Output(UInt(retireWidth.W))
  val store_retired = Output(UInt(retireWidth.W))
  val amo_retired = Output(UInt(retireWidth.W))
  val fp_retired = Output(UInt(retireWidth.W))

  val topdown_slots = Output(UInt(log2Ceil(retireWidth + 1).W))
  val topdown_retiring_slots = Output(UInt(log2Ceil(retireWidth + 1).W))
  val topdown_frontend_bound_slots = Output(UInt(log2Ceil(retireWidth + 1).W))
  val topdown_backend_bound_slots = Output(UInt(log2Ceil(retireWidth + 1).W))
  val topdown_badspec_bound_slots = Output(UInt(log2Ceil(retireWidth + 1).W))
}

class BoomPerfHPMEvents(implicit p: Parameters)
  extends BoomModule()(p)
  with HasBoomCoreParameters
{
  val io = IO(new BoomPerfHPMEventsIO)

  private def any(mask: UInt, hits: UInt): Bool = {
    (mask & hits).orR
  }


  // Instruction type
  val committed = io.commit.arch_valids

  io.jump_retired := VecInit((0 until retireWidth).map { w =>
    committed(w) &&
      (io.commit.uops(w).is_jal || io.commit.uops(w).is_jalr)
  }).asUInt

  io.branch_retired := VecInit((0 until retireWidth).map { w =>
    committed(w) &&
      io.commit.uops(w).is_br
  }).asUInt

  io.memory_retired := VecInit((0 until retireWidth).map { w =>
    committed(w) &&
      (io.commit.uops(w).uses_ldq ||
      io.commit.uops(w).uses_stq ||
      io.commit.uops(w).is_amo)
  }).asUInt

  io.load_retired := VecInit((0 until retireWidth).map { w =>
    committed(w) &&
      io.commit.uops(w).uses_ldq &&
      !io.commit.uops(w).is_amo
  }).asUInt

  io.store_retired := VecInit((0 until retireWidth).map { w =>
    committed(w) &&
      io.commit.uops(w).uses_stq &&
      !io.commit.uops(w).is_amo
  }).asUInt

  io.amo_retired := VecInit((0 until retireWidth).map { w =>
    committed(w) &&
      io.commit.uops(w).is_amo
  }).asUInt

  io.fp_retired := VecInit((0 until retireWidth).map { w =>
    committed(w) &&
      io.commit.uops(w).fp_val
  }).asUInt

  private val topdownWidth = log2Ceil(retireWidth + 1)

  // Track cycles spent recovering from frontend redirects or ROB flushes.
  val recovering = RegInit(false.B)

  when (io.sfence_valid || io.redirect_val || io.redirect_flush || io.rob_flush) {
    recovering := true.B
  }

  when (io.fetchpacket_valid) {
    recovering := false.B
  }

  //-------------------------------------------------------------
  // Frontend
  //-------------------------------------------------------------

  // A frontend-bound slot is a non-stalled decode slot without a valid fetch bundle.
  val frontend_bound_slots =
    VecInit((0 until coreWidth).map { w =>
      !recovering &&
      !io.dec_stalls(w) &&
      (!io.fetchpacket_valid || !io.dec_fbundle_valids(w))
    })

  //-------------------------------------------------------------
  // Bad speculation
  //-------------------------------------------------------------

  // Track decode minus commit, using decode as a dispatch proxy.
  val decCount = PopCount(io.dec_fire)
  val comCount = PopCount(io.commit_uops)
  val badspecDelta = decCount.zext - comCount.zext

  // Keep ROB-sized signed debt so commit surplus can cancel later decode surplus.
  // TODO: is this overkill?
  val badspecDebtWidth = log2Ceil(numRobEntries + 1) + 1
  val badspec_debt = RegInit(0.S(badspecDebtWidth.W))
  val nextBadspecDebt = badspec_debt + badspecDelta

  val badspec_slots =
    Mux(nextBadspecDebt > retireWidth.S,
      retireWidth.U(topdownWidth.W),
      Mux(nextBadspecDebt > 0.S,
        nextBadspecDebt.asUInt.pad(topdownWidth),
        0.U(topdownWidth.W)))

  badspec_debt := nextBadspecDebt - badspec_slots.zext

  //-------------------------------------------------------------
  // Top-down slots
  //-------------------------------------------------------------

  io.topdown_slots := retireWidth.U(topdownWidth.W)
  io.topdown_retiring_slots := PopCount(io.commit_uops).pad(topdownWidth)
  io.topdown_frontend_bound_slots := PopCount(frontend_bound_slots).pad(topdownWidth)
  io.topdown_badspec_bound_slots := badspec_slots

  val usedTopdownSlots =
    io.topdown_retiring_slots +
    io.topdown_frontend_bound_slots +
    io.topdown_badspec_bound_slots

  io.topdown_backend_bound_slots :=
    Mux(io.topdown_slots > usedTopdownSlots,
      io.topdown_slots - usedTopdownSlots,
      0.U(topdownWidth.W))

  def perfEvents: EventSets = new EventSets(Seq(
    // Let's make this be instruction type counters for now.
    new EventSet(any, Seq(
      ("inst_retired.jump",   () => io.jump_retired),
      ("inst_retired.branch", () => io.branch_retired),
      ("inst_retired.memory", () => io.memory_retired),
      ("inst_retired.load",   () => io.load_retired),
      ("inst_retired.store",  () => io.store_retired),
      ("inst_retired.amo",    () => io.amo_retired),
      ("inst_retired.fp",     () => io.fp_retired)
    )),
    new EventSet(any, Seq(
      ("nop",                                () => false.B.asUInt),
      ("branch misprediction",              () => io.branch_mispredict.asUInt),
      ("control-flow target misprediction", () => io.cfi_target_mispredict.asUInt),
      ("flush",                             () => io.rob_flush.asUInt),
      ("branch resolved",                   () => io.branch_resolved.asUInt)
    )),

    new EventSet(any, Seq(
      ("I$ miss",     () => io.ifu_acquire.asUInt),
      ("D$ miss",     () => io.lsu_acquire.asUInt),
      ("D$ release",  () => io.lsu_release.asUInt),
      ("ITLB miss",   () => io.ifu_tlb_miss.asUInt),
      ("DTLB miss",   () => io.lsu_tlb_miss.asUInt),
      ("L2 TLB miss", () => io.ptw_l2_miss.asUInt)
    )),

    new EventSet(any, Seq(
      ("TOPDOWN.SLOTS",                 () => io.topdown_slots),
      ("TOPDOWN.RETIRING.SLOTS",        () => io.topdown_retiring_slots),
      ("TOPDOWN.FRONTEND_BOUND.SLOTS",  () => io.topdown_frontend_bound_slots),
      ("TOPDOWN.BACKEND_BOUND.SLOTS",   () => io.topdown_backend_bound_slots),
      ("TOPDOWN.BAD_SPECULATION.SLOTS", () => io.topdown_badspec_bound_slots)
    ))
  ))
}
