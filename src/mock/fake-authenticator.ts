import EventEmitter from 'eventemitter3';
import Authenticator, { AuthenticatorEventType } from '../components/auth/authenticator';
import { User } from '../components/auth/user';

export default class FakeAuthenticator extends EventEmitter<AuthenticatorEventType>
  implements Authenticator {
  readonly user: User = {
    username: 'fakeUser',
    sossToken: '',
  };

  async init(): Promise<void> {}

  login(): Promise<never> {
    throw new Error('Method not implemented.');
  }

  logout(): Promise<never> {
    throw new Error('Method not implemented.');
  }
}
