import * as msgpack from '@msgpack/msgpack';
import * as assert from 'assert';
import WebSocket from 'ws';
import logger, { CustomLogger } from './logger';
import { WebSocketMiddleware } from './websocket-connect';

export { CustomLogger as Logger };

export interface RpcRequest<T = unknown> {
  version: string;
  method: string;
  params?: T;
  id?: string | number | null;
}

export interface RpcError {
  code: number;
  message: string;
}

export interface RpcResponse<T = unknown> {
  version: string;
  result?: T;
  more?: boolean;
  error?: RpcError;
  id: string | number;
}

export interface Sender<T = unknown> {
  socket: WebSocket;
  send(data: T): void;
  end(data: T): void;
  error(error: RpcError): void;
}

export type RpcHandler<Param = any, Result = unknown> = (
  params: Param,
  sender: Sender<Result>,
) => Promise<Result | void> | Result | void;

export default class ApiGateway {
  middleware: WebSocketMiddleware = (socket, data, next) => {
    this._onMessage(socket, data, next);
  };

  registerHandler(method: string, cb: RpcHandler): void {
    this._rpcHandlers[method] = cb;
    logger.info(`registered handler for "${method}"`);
  }

  getLogger(name: string): CustomLogger {
    return logger.child({ tag: name });
  }

  private _rpcHandlers: Record<string, RpcHandler> = {};

  private async _onMessage(
    socket: WebSocket,
    data: WebSocket.Data,
    next: () => void,
  ): Promise<void> {
    assert.ok(data instanceof Buffer);
    const req = msgpack.decode(data) as RpcRequest;
    assert.strictEqual('0', req.version);

    const buildResponse = (response: Partial<RpcResponse>) => {
      return {
        version: '0',
        id: req.id,
        ...response,
      };
    };

    const sender: Sender = {
      socket,
      send: (data) =>
        req.id !== undefined &&
        req.id !== null &&
        socket.send(msgpack.encode(buildResponse({ result: data, more: true }))),
      end: (data) =>
        req.id !== undefined &&
        req.id !== null &&
        socket.send(msgpack.encode(buildResponse({ result: data }))),
      error: (error) =>
        req.id !== undefined &&
        req.id !== null &&
        socket.send(msgpack.encode(buildResponse({ error }))),
    };

    try {
      const handler = this._rpcHandlers[req.method];
      if (!handler) {
        logger.warn(`no handler for method "${req.method}`);
        sender.error({
          code: 1,
          message: 'no such method',
        });
        return;
      }

      const handlerRet = await handler(req.params, sender);
      if (handlerRet === undefined) {
        if (this._rpcHandlers[req.method].length === 1) {
          sender.end(null);
        }
      } else {
        if (this._rpcHandlers[req.method].length === 1) {
          sender.end(handlerRet);
        } else {
          sender.send(handlerRet);
        }
      }
    } catch (e) {
      sender.error({ code: 1, message: e.message });
    }
    next();
  }
}
