import EventEmitter from 'eventemitter3';
import { Authenticator, AuthenticatorEventType } from './authenticator';
export default class StubAuthenticator
  extends EventEmitter<AuthenticatorEventType>
  implements Authenticator {
  readonly user = 'stub';
  readonly token?: string;
  init(): Promise<void>;
  login(): Promise<never>;
  logout(): Promise<never>;
  refreshToken(): Promise<void>;
}
