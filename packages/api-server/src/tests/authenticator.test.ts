import * as jwt from 'jsonwebtoken';
import WebSocket from 'ws';
import auhthenticator from '../authenticator';
import WebSocketConnect, { WebSocketMiddleware } from '../websocket-connect';

describe('Authenticator', () => {
  let app: WebSocketConnect;
  let server: WebSocket.Server;
  let address: WebSocket.AddressInfo;

  beforeEach(() => {
    server = new WebSocket.Server({ port: 0 });
    address = server.address() as WebSocket.AddressInfo;
    app = new WebSocketConnect(server);
    app.use(auhthenticator({ type: 'secret', secretOrPublicKey: 'test' }));
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
});
