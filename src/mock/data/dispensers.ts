export default function fakeDispensers(): Record<string, string[]> {
  /**
   * nameOfPlace: [dispenser, available, on, place]
   */
  return {
    pantry: ['coke_dispenser'],
    hardware_2: ['coke_ingestor'],
    // Add here a new places with its dispensers
  };
}
