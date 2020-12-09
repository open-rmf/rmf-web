import WebSocket from 'ws';
export declare type WebSocketMiddleware = (
  socket: WebSocket,
  data: WebSocket.Data,
  next: () => void,
) => void;
export default class WebSocketConnect {
  server: WebSocket.Server;
  stack: WebSocketMiddleware[];
  constructor(server: WebSocket.Server);
  use(mdw: WebSocketMiddleware): void;
}
