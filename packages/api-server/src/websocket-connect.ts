import WebSocket from 'ws';
import baseLogger from './logger';

export type WebSocketMiddleware = (
  socket: WebSocket,
  data: WebSocket.Data,
  next: () => void,
) => void;

export default class WebSocketConnect {
  stack: WebSocketMiddleware[] = [];

  constructor(public server: WebSocket.Server) {
    server.on('connection', (socket, req) => {
      const logger = baseLogger.child({ tag: req.socket.remoteAddress });
      logger.info('connection established');

      socket.on('message', async (data) => {
        try {
          for (let mdw of this.stack) {
            await new Promise((res) => mdw(socket, data, res));
          }
        } catch (e) {
          logger.info(`[${req.socket.remoteAddress}] ${e}`);
          socket.close();
          logger.info('connection closed');
        }
      });
    });
  }

  use(mdw: WebSocketMiddleware) {
    this.stack.push(mdw);
  }
}
