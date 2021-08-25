import EventEmitter from 'eventemitter3';
import { Authenticator, AuthenticatorEventType } from './authenticator';

export class StubAuthenticator
  extends EventEmitter<AuthenticatorEventType>
  implements Authenticator {
  readonly user: string;

  readonly token?: string;

  constructor(user = 'stub', token: string | undefined = undefined) {
    super();
    this.user = user;
    this.token = token;
  }

  init(): Promise<void> {
    return Promise.resolve();
  }

  login(): Promise<never> {
    throw new Error('not supported');
  }

  logout(): Promise<never> {
    throw new Error('not supported');
  }

  refreshToken(): Promise<void> {
    return Promise.resolve();
  }
}

export default StubAuthenticator;
