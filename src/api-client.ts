import * as msgpack from '@msgpack/msgpack';
import { RpcRequest, RpcResponse } from '@osrf/romi-js-api-server/src/api-gateway';
import Debug from 'debug';

const debug = Debug('ApiClient');

export default class ApiClient {
  static async connect(url: string): Promise<ApiClient> {
    const socket = new WebSocket(url);
    socket.binaryType = 'arraybuffer';
    await new Promise((res) => socket.addEventListener('open', res));
    return new ApiClient(socket);
  }

  rpcRequest<Result = unknown, Param = unknown>(
    method: string,
    params: Param,
    responseCallback: (resp: Result) => void,
  ): void {
    const request = this._buildRpcRequest(method, params);
    debug(`sending rpc request for "${request.method}", with id "${request.id}"`);
    this.socket.send(msgpack.encode(request));
    if (request.id !== undefined && request.id !== null) {
      this._ongoingRequests[request.id] = responseCallback as any;
    }
  }

  rpcNotify<Param = unknown>(method: string, params: Param): void {
    const request = this._buildRpcRequest(method, params, true);
    debug(`sending rpc notify for "${request.method}"`);
    this.socket.send(msgpack.encode(request));
  }

  private _idCounter = 0;
  private _ongoingRequests: Record<string | number, (result: unknown) => void> = {};

  private constructor(public socket: WebSocket) {
    socket.addEventListener('message', (ev) => this._onMessage(ev.data));
  }

  private _buildRpcRequest<T>(method: string, params: T, isNotify = false): RpcRequest<T> {
    const request: RpcRequest<T> = {
      version: '0',
      method,
      params,
    };
    // undefined gets serialized as null by msgpack, to make id not in the payload, we have to make
    // the object to not have the field at all.
    if (!isNotify) {
      request.id = this._idCounter++;
    }
    return request;
  }

  private _onMessage(data: any): void {
    const resp = msgpack.decode(data) as RpcResponse;
    if (resp.error) {
      console.error(resp.error.message);
      return;
    }
    debug(`receive rpc response for id "${resp.id}"`);
    const cb = this._ongoingRequests[resp.id];
    cb && cb(resp.result);
    if (!resp.more) {
      debug(`received final response for id "${resp.id}"`);
      delete this._ongoingRequests[resp.id];
    }
  }
}
