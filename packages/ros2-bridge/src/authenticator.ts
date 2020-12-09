import * as fs from 'fs';
import * as jwt from 'jsonwebtoken';
import WebSocket from 'ws';
import { WebSocketMiddleware } from './websocket-connect';

export interface Options {
  type: 'secret' | 'publicKey';
  secretOrPublicKey: string;
}

export default function authenticator(options: Options): WebSocketMiddleware {
  const secretOrPublicKey = (() => {
    if (options.type === 'publicKey') {
      return fs.readFileSync(options.secretOrPublicKey);
    } else {
      return options.secretOrPublicKey;
    }
  })();

  return (socket: WebSocket & { authorized?: boolean }, data, next) => {
    if (socket.authorized) {
      next();
    }

    if (typeof data !== 'string') {
      throw new Error('expected token');
    }

    jwt.verify(data, secretOrPublicKey); // will throw if invalid
    socket.authorized = true;
    socket.send('ok');
    // don't call `next()`, authentication message should not be chained to downstream middlewares.
  };
}
