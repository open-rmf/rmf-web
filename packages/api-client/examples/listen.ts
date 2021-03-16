import { makeClient } from '../lib';

const client = makeClient('http://localhost:8000');
client.emit('subscribe', 'building_map');
client.emit('subscribe', 'door_states');
client.on('building_map', () => console.log('got building map'));
client.on('door_states', console.log);
