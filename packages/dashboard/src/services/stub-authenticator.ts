import EventEmitter from 'eventemitter3';

import { Authenticator, AuthenticatorEventType } from './authenticator';

/**
 * Hardcoded token using the secret 'rmfisawesome', expires in 2035-01-01.
 * To update the token, use https://jwt.io and paste in the payload, also remember
 * to set the secret to `rmfisawesome`.
 *
 * header:
 * {
 *   "alg": "HS256",
 *   "typ": "JWT"
 * }
 * payload:
 * {
 *   "sub": "stub",
 *   "preferred_username": "admin",
 *   "iat": 1516239022,
 *   "aud": "rmf_api_server",
 *   "iss": "stub",
 *   "exp": 2051222400
 * }
 */
const ADMIN_TOKEN =
  'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOiJzdHViIiwicHJlZmVycmVkX3VzZXJuYW1lIjoiYWRtaW4iLCJpYXQiOjE1MTYyMzkwMjIsImF1ZCI6InJtZl9hcGlfc2VydmVyIiwiaXNzIjoic3R1YiIsImV4cCI6MjA1MTIyMjQwMH0.zzX3zXp467ldkzmLVIadQ_AHr8M5uWVV43n4wEB0OhE';

// same as the admin token, except the `preferred_username` is "user".
const USER_TOKEN =
  'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOiJzdHViIiwicHJlZmVycmVkX3VzZXJuYW1lIjoidXNlciIsImlhdCI6MTUxNjIzOTAyMiwiYXVkIjoicm1mX2FwaV9zZXJ2ZXIiLCJpc3MiOiJzdHViIiwiZXhwIjoyMDUxMjIyNDAwfQ.vK3n4FbshCykQ9BW49w_7AfqKgbN9j2R3-Qh-rIOt_g';

export class StubAuthenticator
  extends EventEmitter<AuthenticatorEventType>
  implements Authenticator
{
  readonly user: string;

  readonly token?: string;

  constructor(isAdmin = true) {
    super();
    this.user = isAdmin ? 'admin' : 'user';
    this.token = isAdmin ? ADMIN_TOKEN : USER_TOKEN;
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
