import Authenticator from '../components/auth/authenticator';

export default class FakeAuthenticator implements Authenticator {
  authenticated = true;
  sossToken = '';

  login(): Promise<never> {
    throw new Error('Method not implemented.');
  }

  logout(): Promise<never> {
    throw new Error('Method not implemented.');
  }
}
