import EventEmitter from 'eventemitter3';

import { Authenticator, AuthenticatorEventType } from './authenticator';

export class StubAuthenticator
  extends EventEmitter<AuthenticatorEventType>
  implements Authenticator
{
  readonly user: string;

  readonly token?: string;

  constructor() {
    super();
    this.user = 'stub';
    // hardcoded token that expires in 2035-01-01
    this.token =
      'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOiJzdHViIiwicHJlZmVycmVkX3VzZXJuYW1lIjoic3R1YiIsImlhdCI6MTUxNjIzOTAyMiwiYXVkIjoicm1mX2FwaV9zZXJ2ZXIiLCJpc3MiOiJzdHViIiwiZXhwIjoyMDUxMjIyNDAwfQ.dN7xOpbeN2A4QXM8Mmc2ZzWqC8w1XNm8IYsJ0FdeKCc';
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
