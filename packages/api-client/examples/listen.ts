import { SioClient } from '../lib';

const client = new SioClient('http://localhost:8000');
const l1 = client.subscribeDoorState('coe_door', console.log);
const l2 = client.subscribeDoorState('coe_door', console.log);

client.unsubscribe(l1);
client.unsubscribe(l2);
