import * as http from 'http';
import WebSocket from 'ws';
import baseLogger from './logger';

export type WebSocketMiddleware = (
  socket: WebSocket,
  req: http.IncomingMessage,
  next: () => void,
) => void;

export default class WebSocketConnect {
  stack: WebSocketMiddleware[] = [];

  constructor(public server: WebSocket.Server) {
    server.on('connection', async (socket, req) => {
      const logger = baseLogger.child({ tag: req.socket.remoteAddress });
      logger.info('connection established');

      try {
        for (let mdw of this.stack) {
          await new Promise<void>((res) => mdw(socket, req, res));
        }
      } catch (e) {
        logger.info(e);
        socket.close();
        logger.info('connection closed');
      }
    });
  }

  use(mdw: WebSocketMiddleware) {
    this.stack.push(mdw);
  }
}
