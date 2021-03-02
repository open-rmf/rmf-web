import { make_client, topics } from '../lib';

const client = make_client('http://localhost:8000');
client.emit('subscribe', topics.buildingMap);
client.emit('subscribe', topics.doorStates);
client.on('building_map', () => console.log('got building map'));
client.on('door_states', console.log);
