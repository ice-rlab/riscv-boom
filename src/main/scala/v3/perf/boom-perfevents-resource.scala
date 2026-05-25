package boom.v3.perf

/*
 * Description of one BOOM performance event.
 *
 * BOOM/Rocket EventSets use:
 *
 *   bits [7:0] = EventSet index
 *   bit  [8+n] = event bit n within that EventSet
 */
case class BoomPerfEventMeta(
  name: String,
  set: Int,
  bit: Int
) {
  require(set >= 0, s"Invalid BOOM EventSet index: $set")
  require(set < 256, s"BOOM EventSet index must fit in 8 bits: $set")
  require(bit >= 0, s"Invalid BOOM event bit: $bit")

  /*
   * Actual BOOM hardware mhpmevent selector.
   *
   * Examples:
   *
   *   set 0, bit 0 => 0x100
   *   set 1, bit 0 => 0x101
   *   set 1, bit 1 => 0x201
   *   set 2, bit 0 => 0x102
   */
  def encoding: BigInt =
    BigInt(set) | (BigInt(1) << (8 + bit))
}

/*
 * BOOM performance-event definitions.
 *
 * The ordering of these sets and events must exactly match the EventSets
 * instantiated by the BOOM RTL.
 */
object BoomPerfEvents {

  private val eventNames: Seq[Seq[String]] = Seq(
    /*
     * EventSet 0
     *
     * Reserved placeholder events.
     */
    Seq(
      "nop",
      "nop",
      "nop",
      "nop"
    ),

    /*
     * EventSet 1
     */
    Seq(
      "nop",
      "branch misprediction",
      "control-flow target misprediction",
      "flush",
      "branch resolved"
    ),

    /*
     * EventSet 2
     */
    Seq(
      "I$ miss",
      "D$ miss",
      "D$ release",
      "ITLB miss",
      "DTLB miss",
      "L2 TLB miss"
    ),

    /*
     * EventSet 3
     *
     * Top-Down Microarchitectural Analysis slot events.
     */
    Seq(
      "TOPDOWN.SLOTS",
      "TOPDOWN.RETIRING.SLOTS",
      "TOPDOWN.FRONTEND_BOUND.SLOTS",
      "TOPDOWN.BACKEND_BOUND.SLOTS",
      "TOPDOWN.BAD_SPECULATION.SLOTS"
    )
  )

  val eventSets: Seq[Seq[BoomPerfEventMeta]] =
    eventNames.zipWithIndex.map { case (names, set) =>
      names.zipWithIndex.map { case (name, bit) =>
        BoomPerfEventMeta(
          name = name,
          set = set,
          bit = bit
        )
      }
    }

  val allEvents: Seq[BoomPerfEventMeta] =
    eventSets.flatten

  /*
   * Raw BOOM hardware mhpmevent selector values.
   *
   * These should be emitted through:
   *
   *   riscv,raw-event-to-mhpmcounters
   */
  def rawPmuSelectors: Seq[BigInt] =
    allEvents
      .map(_.encoding)
      .distinct
      .sorted

  /*
   * Compatibility mapping for PerfEventsResource.bind.
   *
   * PerfEventsResource.bind ignores the first tuple field and treats the
   * second field as the raw mhpmevent selector.
   */
  def pmuMappings: Seq[(BigInt, BigInt)] =
    rawPmuSelectors.map { selector =>
      selector -> selector
    }

  def names(set: Int): Seq[String] = {
    require(
      set >= 0 && set < eventSets.length,
      s"Invalid BOOM EventSet index: $set"
    )

    eventSets(set).map(_.name)
  }
}
