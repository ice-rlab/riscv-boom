package boom.v3.prof

import chisel3._
import chisel3.util._

import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.util._

import boom.v3.common._
import boom.v3.util._
import boom.v3.exu.{CommitSignals, BrResolutionInfo}

// TODO: Make sure compiles without CTR (should be good now)
// TODO: Return? --> Return gets translated to jalr, so it should be fine.
// TODO: Handle mispredictions --> Forward how the the bracnh was predicted into the CTR?
// TODO: Abstract CTR class
// TODO: Handle exceptions? Or other control flow pseudo instructions (not super performance critical right now)
// TODO: Remove valid from the registers --> no valid signal should just have address zero, software can filter out
// TODO: How can we move this around different stages in the pipeleine?
// TODO: How to interact with events? 
// TODO: Right now the way we get the address is from debug_pc, wich is not synthesizable. In future we have to reconstruct the PC


// TODO: How to emulate something like PEBS how should this interact with ?

class CTREntry(implicit p: Parameters) extends BoomBundle {
  // val from = UInt(vaddrBitsExtended.W)
  // val to = UInt(vaddrBitsExtended.W)
  val valid = Bool()
  val from = UInt(vaddrBitsExtended.W)
  val to = UInt(vaddrBitsExtended.W)
  val m = Bool() // Was this mispredicted?
}

class CTRcfg(implicit p: Parameters) extends BoomBundle {
  val en = Bool()
  val clr = Bool()
}

class CTRIo(implicit p: Parameters) extends BoomBundle {
  val commit = Input(new CommitSignals())
  val ctr_entries = Output(Vec(nCTREntries, new CTREntry()))
  val cfg = Input(new CTRcfg())
  val full = Output(Bool())
}

// Is implemented as an N-entry shift register.
class CTR(implicit p: Parameters) extends BoomModule {
  val io = IO(new CTRIo())

  // Check all CTR signals and filter out all retired uops
  val is_retiring = VecInit((0 until retireWidth).map { i =>
    io.commit.valids(i)
  })

  // Always store the last uop that is retiring.
  val last_uop = RegInit(0.U.asTypeOf(new MicroOp()))

  when (is_retiring.asUInt =/= 0.U) {
    last_uop := PriorityMux(is_retiring, io.commit.uops)
  }

  val commitPairs: Seq[(MicroOp, Bool)] =
    io.commit.uops.zip(io.commit.valids)

  val uops: Seq[(MicroOp, Bool)] = (last_uop, true.B) +: commitPairs

  val is_first_cfi: Seq[Bool] = uops
    .dropRight(1)
    .map { case (u, v) =>
      v && ((u.is_br && u.taken) || u.is_jal || u.is_jalr) // TODO: sfb?
    }

  val is_new_ctr_entry: Seq[Bool] =
    is_first_cfi.zipWithIndex.map { case (cfi, i) =>
      cfi && uops(i + 1)._2
    }

  // New entries in this cycle, with valid folded into the entry itself
  val rawNew = VecInit((0 until retireWidth).map { i =>
    val e = Wire(new CTREntry())
    val fire = io.cfg.en && is_new_ctr_entry(i)

    e.valid := fire
    e.from  := Mux(fire, uops(i)._1.debug_pc, 0.U)
    e.to    := Mux(fire, uops(i + 1)._1.debug_pc, 0.U)
    e.m     := false.B // TODO: hook up mispredict

    e
  })

  // CTR entries register file
  val entries = RegInit(
    VecInit.fill(nCTREntries) {
      0.U.asTypeOf(new CTREntry()) // valid = false, from/to/m = 0
    }
  )

  // Full when the last entry is valid
  io.full := entries.last.valid

  val nNew = PopCount(rawNew.map(_.valid))

  val entriesNext = Wire(Vec(nCTREntries, new CTREntry()))
  // Default to zero; we overwrite below
  for (i <- 0 until nCTREntries) {
    entriesNext(i) := 0.U.asTypeOf(new CTREntry())
  }

  val idxLast = Wire(Vec(nCTREntries, UInt(log2Ceil(nCTREntries).W)))
  for (i <- 0 until nCTREntries) {
    idxLast(i) := i.U - nNew // No wrap-around needed in current scheme
  }

  // Shift in new entries at the front, slide old ones down
  for (i <- 0 until retireWidth) {
    when (i.U < nNew) {
      entriesNext(i) := rawNew(i)
    }.otherwise {
      entriesNext(i) := entries(idxLast(i))
    }
  }

  for (i <- retireWidth until nCTREntries) {
    entriesNext(i) := entries(idxLast(i))
  }

  // Clear or update
  when (io.cfg.clr) {
    entries := VecInit.fill(nCTREntries) {
      0.U.asTypeOf(new CTREntry())
    }
  }.elsewhen (nNew =/= 0.U) {
    entries := entriesNext
  }

  // Expose full entries (including .valid bit) to CSR side
  io.ctr_entries := entries

  dontTouch(entries)

  override def toString: String = BoomCoreStringPrefix(
    "==CTR==",
    "CTR Entries        : " + nCTREntries,
    "CTR entry width    : " + new CTREntry().getWidth + " bits"
  )
}