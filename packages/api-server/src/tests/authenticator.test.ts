import * as jwt from 'jsonwebtoken';
import WebSocket from 'ws';
import authenticator from '../authenticator';
import WebSocketConnect, { WebSocketMiddleware } from '../websocket-connect';

describe('Authenticator', () => {
  let app: WebSocketConnect;
  let server: WebSocket.Server;
  let address: WebSocket.AddressInfo;

  beforeEach(() => {
    server = new WebSocket.Server({ port: 0 });
    address = server.address() as WebSocket.AddressInfo;
    app = new WebSocketConnect(server);
    app.use(authenticator({ type: 'secret', secretOrPublicKey: 'test' }));
  });

  afterEach(() => {
    server.close();
  });

  test('allows valid oauth token', (done) => {
    const mdw: WebSocketMiddleware = jest.fn(() => {
      done();
    });
    app.use(mdw);

    const client = new WebSocket(`ws://localhost:${address.port}`);
    client.on('open', () => {
      client.send(jwt.sign('test', 'test'));
      client.send('test');
    });
  }, 100);

  test('reject invalid oauth token', (done) => {
    const client = new WebSocket(`ws://localhost:${address.port}`);
    // Should close connection on bad token
    client.once('close', () => done());
    client.on('open', () => {
      client.send('bad token');
    });
  });

  test.only('reject binary payload', (done) => {
    const client = new WebSocket(`ws://localhost:${address.port}`);
    // Should close connection on bad token
    client.once('close', () => done());
    client.on('open', () => {
      client.send(Buffer.from([1, 2, 3]));
    });
  });
});
