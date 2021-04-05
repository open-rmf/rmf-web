const { execSync } = require('child_process');
const { rmdirSync } = require('fs');

rmdirSync(`${__dirname}/lib`, { recursive: true });
const rmfMsgs = [
  'rmf_building_map_msgs',
  'rmf_charger_msgs',
  'rmf_door_msgs',
  'rmf_lift_msgs',
  'rmf_dispenser_msgs',
  'rmf_ingestor_msgs',
  'rmf_fleet_msgs',
  'rmf_task_msgs',
];
execSync(`python3 -m pipenv run python -m ts_ros -o lib/ ${rmfMsgs.join(' ')}`, {
  stdio: 'inherit',
});
