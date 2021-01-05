import Debug from 'debug';
import EventEmitter from 'eventemitter3';
import Keycloak_, { KeycloakInstance } from 'keycloak-js';
import { BASE_PATH } from '../../util/url';
import { User } from './user';

const debug = Debug('authenticator');

export type AuthenticatorEventType = {
  userChanged: [User | null];
};

export default interface Authenticator extends EventEmitter<AuthenticatorEventType> {
  readonly user?: User;
  readonly token?: string;

  /**
   * It is required to call this before using any of the authenticator functions.
   */
  init(): Promise<void>;

  /**
   * Note: This redirects to external login page so it will never return.
   */
  login(): Promise<never>;

  /**
   * Note: This redirects to external logout page so it will never return.
   */
  logout(): Promise<void>;
}

export type AuthConfig<T = typeof Keycloak_> = T extends (config: infer U) => unknown ? U : never;

export class DefaultAuthenticator
  extends EventEmitter<AuthenticatorEventType>
  implements Authenticator {
  get user(): User | undefined {
    return this._user;
  }

  get token(): string | undefined {
    return this._inst.idToken;
  }

  constructor(config: AuthConfig, redirectUri?: string) {
    super();
    this._inst = Keycloak_(config);
    this._redirectUri = redirectUri;
  }

  async init() {
    debug('initializing authenticator');

    this._inst.onAuthSuccess = async () => {
      this._user = {
        username: (this._inst.idTokenParsed as any).preferred_username,
      };
      debug('authenticated as', this._user.username);
      this.emit('userChanged', this._user);
    };

    this._inst.onAuthLogout = () => {
      debug('logout');
      this._user = undefined;
      this.emit('userChanged', null);
    };

    await this._inst.init({
      onLoad: 'check-sso',
      silentCheckSsoRedirectUri: `${BASE_PATH}/silent-check-sso.html`,
      redirectUri: this._redirectUri,
    });
    try {
      const refreshed = await this._inst.updateToken(30);
      refreshed && debug('token refreshed');
    } catch {}

    this._user = this._inst.idTokenParsed && {
      username: (this._inst.idTokenParsed as any).preferred_username,
    };
  }

  async login(): Promise<never> {
    await this._inst.login();
    throw new Error('should not reach here');
  }

  async logout(): Promise<never> {
    await this._inst.logout();
    throw new Error('should not reach here');
  }

  private _inst: KeycloakInstance;
  private _redirectUri?: string;
  private _user?: User;
}
