import EventEmitter from 'eventemitter3';
import { Authenticator, AuthenticatorEventType } from '../lib/authenticator';
import { User } from '../lib/user';

export class FakeAuthenticator
  extends EventEmitter<AuthenticatorEventType>
  implements Authenticator {
  user?: User;

  constructor(user?: User) {
    super();
    this.user = user;
  }

  async init(): Promise<void> {
    // It is required to call this before using any of the authenticator functions.
  }

  login(): Promise<never> {
    throw new Error('Method not implemented.');
  }

  logout(): Promise<never> {
    throw new Error('Method not implemented.');
  }

  refreshToken(): Promise<void> {
    throw new Error('not supported');
  }
}

export default FakeAuthenticator;
