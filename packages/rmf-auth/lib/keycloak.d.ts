import EventEmitter from 'eventemitter3';
import { AuthConfig, Authenticator, AuthenticatorEventType } from './authenticator';
export default class KeycloakAuthenticator
  extends EventEmitter<AuthenticatorEventType>
  implements Authenticator {
  get user(): string | undefined;
  get token(): string | undefined;
  constructor(config: AuthConfig, redirectUri?: string);
  private _getUser;
  init(): Promise<void>;
  refreshToken(): Promise<void>;
  login(successRedirectUri: string): Promise<never>;
  logout(): Promise<never>;
  private _initialized;
  private _inst;
  private _redirectUri?;
  private _user?;
}
