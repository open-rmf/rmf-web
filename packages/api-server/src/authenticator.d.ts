import { WebSocketMiddleware } from './websocket-connect';
export interface Options {
  type: 'secret' | 'publicKey';
  secretOrPublicKey: string;
}
export default function authenticator(options: Options): WebSocketMiddleware;
