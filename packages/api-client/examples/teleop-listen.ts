import { SioClient } from '../lib';

const client = new SioClient('http://localhost:8000');
const l1 = client.subscribeTeleoperationVideoJoin('tinyRobot1', console.log);
const l2 = client.subscribeTeleoperationVideoLeave('tinyRobot1', console.log);
const l3 = client.subscribeTeleoperationMotion('tinyRobot1', console.log);
