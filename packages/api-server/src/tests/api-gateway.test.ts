import * as msgpack from '@msgpack/msgpack';
import WebSocket from 'ws';
import ApiGateway, { RpcRequest, RpcResponse } from '../api-gateway';
import WebSocketConnect from '../websocket-connect';

function echo(params: any) {
  return params;
}

function makeRpcRequest(method: string, params?: any, id?: string | number | null): Uint8Array {
  const rpcRequest: RpcRequest = {
    version: '0',
    method,
  };
  if (params !== undefined) {
    rpcRequest.params = params;
  }
  if (id !== undefined) {
    rpcRequest.id = id;
  }
  return msgpack.encode(rpcRequest);
}

let app: WebSocketConnect;
let server: WebSocket.Server;
let url: string;
let api: ApiGateway;

beforeEach(async () => {
  server = new WebSocket.Server({ host: 'localhost', port: 0 });
  await new Promise((res) => server.once('listening', res));
  const port = (server.address() as WebSocket.AddressInfo).port;
  url = `ws://localhost:${port}`;
  app = new WebSocketConnect(server);
  api = new ApiGateway();
  app.use(api.middleware);
});

afterEach(() => {
  server.close();
});

test('primitive request params', (done) => {
  api.registerHandler('test', (params: string) => {
    expect(params).toBe('hello');
    return 'world';
  });

  const client = new WebSocket(url);
  client.on('message', (data: Buffer) => {
    expect(data).toBeInstanceOf(Buffer);
    const resp = msgpack.decode(data) as RpcResponse;
    expect(resp.version).toBe('0');
    expect(resp.id).toBe(0);
    expect(resp.error).toBeUndefined();
    expect(resp.more).toBeUndefined();
    expect(resp.result).toBe('world');
    done();
  });
  client.once('open', () => client.send(makeRpcRequest('test', 'hello', 0)));
});

test('object request params', (done) => {
  api.registerHandler('test', (params: { data: string }) => {
    expect(params.data).toBe('hello');
    return { data: 'world' };
  });

  const client = new WebSocket(url);
  client.on('message', (data: Buffer) => {
    expect(data).toBeInstanceOf(Buffer);
    const resp = msgpack.decode(data) as RpcResponse;
    expect(resp.version).toBe('0');
    expect(resp.id).toBe(0);
    expect(resp.error).toBeUndefined();
    expect(resp.more).toBeUndefined();
    expect((resp.result as any).data).toBe('world');
    done();
  });
  client.once('open', () => client.send(makeRpcRequest('test', { data: 'hello' }, 0)));
});

test('request using string id results in a response with string id', (done) => {
  api.registerHandler('test', echo);

  const client = new WebSocket(url);
  client.on('message', (data: Buffer) => {
    expect(data).toBeInstanceOf(Buffer);
    const resp = msgpack.decode(data) as RpcResponse;
    expect(resp.id).toBe('0');
    done();
  });
  client.once('open', () => client.send(makeRpcRequest('test', 'hello', '0')));
});

[undefined, null].forEach((id) => {
  test(`request with ${id} does not return any results`, (done) => {
    api.registerHandler('test', echo);

    const client = new WebSocket(url);
    client.on('message', () => {
      fail('should not receive response');
    });
    client.once('open', () => client.send(makeRpcRequest('test', 'hello', id)));

    setTimeout(done, 100);
  }, 1000);
});

test('rpc error', (done) => {
  api.registerHandler('test', () => {
    throw new Error('test error');
  });

  const client = new WebSocket(url);
  client.on('message', (data: Buffer) => {
    const resp = msgpack.decode(data) as RpcResponse;
    expect(resp.result).toBeUndefined();
    expect(resp.error?.code).toBe(1);
    expect(resp.error?.message).toBe('test error');
    done();
  });
  client.once('open', () => client.send(makeRpcRequest('test', 'hello', 0)));
});

test('async rpc streaming responses', (done) => {
  let even = 0;
  api.registerHandler('testEven', (_, sender) => {
    const t = setInterval(() => {
      if (even >= 10) {
        clearInterval(t);
        sender.end(even);
      } else {
        sender.send(even);
      }
      even += 2;
    }, 100);
  });

  let odd = 1;
  api.registerHandler('testOdd', (_, sender) => {
    const t = setInterval(() => {
      if (odd >= 9) {
        clearInterval(t);
        sender.end(odd);
      } else {
        sender.send(odd);
      }
      odd += 2;
    }, 100);
  });

  const received: number[] = [];
  const client = new WebSocket(url);
  client.on('message', (data: Buffer) => {
    const resp = msgpack.decode(data) as RpcResponse<number>;
    if (resp.result === 9 || resp.result === 10) {
      expect(resp.more).toBeUndefined();
    } else {
      expect(resp.more).toBe(true);
    }
    received.push(resp.result!);
    if (received.length >= 11) {
      for (let i = 0; i <= 10; i++) {
        expect(received[i]).toBe(i);
      }
      done();
    }
  });
  client.once('open', () => {
    client.send(makeRpcRequest('testEven', undefined, 0));
    setTimeout(() => client.send(makeRpcRequest('testOdd', undefined, 1)), 50);
  });
}, 10000);
