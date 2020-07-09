export default function fakePlaces(): Record<string, string[]> {
  /**
   * nameOfFleet: [places, available, to, go]
   */
  return {
    magni: [
      'supplies',
      'magni2_charger',
      'coe',
      'magni1_charger',
      'hardware_2',
      'cubicle_2',
      'pantry',
      'station_1',
      'lounge',
      'cubicle_1',
      'hardware_1',
      'station_2',
    ],
    SuperFleet: [
      'supplies',
      'magni2_charger',
      'coe',
      'magni1_charger',
      'hardware_2',
      'cubicle_2',
      'pantry',
      'station_1',
      'lounge',
      'cubicle_1',
      'hardware_1',
      'station_2',
    ],
    // Add here a new fleet with its places
    //FleetA and FleetB are for testing purposes (loop-form.test.tsx)
    FleetA: ['', 'supplies'],
    FleetB: ['supplies', 'supplies'],
  };
}
