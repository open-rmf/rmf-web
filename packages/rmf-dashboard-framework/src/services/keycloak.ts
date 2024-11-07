import Debug from 'debug';
import EventEmitter from 'eventemitter3';
import Keycloak from 'keycloak-js';

import { Authenticator, AuthenticatorEventType } from './authenticator';

const debug = Debug('authenticator');

export class KeycloakAuthenticator
  extends EventEmitter<AuthenticatorEventType>
  implements Authenticator
{
  get user(): string | undefined {
    return this._user;
  }

  get isAdmin(): boolean | undefined {
    return this._isAdmin;
  }

  get token(): string | undefined {
    return this._inst.token;
  }

  /**
   *
   * @param config
   * @param redirectUri
   * @param silentCheckSsoRedirectUri If provided, enable silent check sso with the provided uri,
   * see https://www.keycloak.org/docs/13.0/securing_apps/index.html#_javascript_adapter for more information.
   */
  constructor(config: Keycloak.KeycloakConfig | string, silentCheckSsoRedirectUri?: string) {
    super();
    this._inst = new Keycloak(config);
    this._silentCheckSsoRedirectUri = silentCheckSsoRedirectUri;
  }

  private _getUser(): string {
    // eslint-disable-next-line @typescript-eslint/no-explicit-any
    return (this._inst.idTokenParsed as any).preferred_username;
  }

  private _isUserAdmin(): boolean {
    if (!this._inst.idTokenParsed) {
      return false;
    }

    const roles: string[] | undefined = (this._inst.idTokenParsed as any).realm_access?.roles;
    if (roles && roles.includes('superuser')) {
      return true;
    }
    return false;
  }

  async init(): Promise<void> {
    if (this._initialized) {
      debug(`${new Date().toLocaleTimeString()}: already initialized`);
      return;
    }

    debug('initializing authenticator');

    this._inst.onAuthSuccess = async () => {
      this._user = this._getUser();
      debug(`${new Date().toLocaleTimeString()}: authenticated as ${this._user}`);
      this.emit('userChanged', this._user);
    };

    this._inst.onAuthLogout = () => {
      debug(`${new Date().toLocaleTimeString()}: logout`);
      this._user = undefined;
      this.emit('userChanged', null);
    };

    await this._inst.init({
      onLoad: 'check-sso',
      silentCheckSsoRedirectUri: this._silentCheckSsoRedirectUri,
    });
    try {
      const refreshed = await this._inst.updateToken(30);
      refreshed && debug(`${new Date().toLocaleTimeString()}: initToken token refreshed`);
    } catch {
      debug(`${new Date().toLocaleTimeString()}: initToken token not refreshed`);
    }

    this._user = this._inst.tokenParsed && this._getUser();
    this._isAdmin = this._isUserAdmin();
    this._initialized = true;
  }

  async refreshToken(): Promise<void> {
    // check and update the token 30 seconds prior to expiry
    if (this._initialized) {
      const refreshed = await this._inst.updateToken(30);
      if (refreshed) {
        this._user = this._getUser();
        this._isAdmin = this._isUserAdmin();
        this.emit('tokenRefresh', null);
      } else {
        debug(`${new Date().toLocaleTimeString()}: refreshToken token not refreshed`);
      }
    }
    return;
  }

  async login(successRedirectUri?: string): Promise<never> {
    await this._inst.login({
      redirectUri: successRedirectUri,
    });
    throw new Error('should not reach here');
  }

  async logout(): Promise<never> {
    await this._inst.logout();
    throw new Error('should not reach here');
  }

  private _initialized = false;
  private _inst: Keycloak;
  private _silentCheckSsoRedirectUri?: string;
  private _user?: string;
  private _isAdmin = false;
}

export default KeycloakAuthenticator;
