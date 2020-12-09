import WebSocket from 'ws';
import { CustomLogger } from './logger';
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
export declare type RpcHandler<Param = any, Result = unknown> = (
  params: Param,
  sender: Sender<Result>,
) => Promise<Result | void> | Result | void;
export default class ApiGateway {
  middleware: WebSocketMiddleware;
  registerHandler(method: string, cb: RpcHandler): void;
  getLogger(name: string): CustomLogger;
  private _rpcHandlers;
  private _onMessage;
}
