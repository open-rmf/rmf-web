import debug from 'debug';
import EventEmitter from 'eventemitter3';
import Keycloak_, { KeycloakInstance } from 'keycloak-js';
import { User } from './user';

export type AuthenticatorEventType = {
  userChanged: [User | null];
};

export default interface Authenticator extends EventEmitter<AuthenticatorEventType> {
  readonly user?: User;
  readonly sossToken?: string;

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

type ConfigType<T = typeof Keycloak_> = T extends (config: infer U) => unknown ? U : never;

export class DefaultAuthenticator extends EventEmitter<AuthenticatorEventType>
  implements Authenticator {
  get user(): User | undefined {
    return this._user;
  }

  get sossToken(): string | undefined {
    return this._inst.idToken;
  }

  constructor(config: ConfigType, redirectUri?: string) {
    super();
    this._inst = Keycloak_(config);
    this._redirectUri = redirectUri;
  }

  async init() {
    debug.log('initializing authenticator');

    this._inst.onAuthSuccess = async () => {
      const profile = this._inst.profile || (await this._inst.loadUserProfile());
      this._user = {
        username: profile.username!,
      };
      debug.log('authenticated as', this._user.username);
      this.emit('userChanged', this._user);
    };

    this._inst.onAuthLogout = () => {
      debug.log('logout');
      this._user = undefined;
      this.emit('userChanged', null);
    };

    const token = getLocalStorage('token');
    const idToken = getLocalStorage('idToken');
    const refreshToken = getLocalStorage('refreshToken');
    await this._inst.init({ redirectUri: this._redirectUri, token, idToken, refreshToken });
    if (this._inst.authenticated) {
      await this._inst.loadUserProfile();
    }
    try {
      const refreshed = await this._inst.updateToken(30);
      refreshed && debug.log('token refreshed');
    } catch {}
    setOrClearLocalStorage('token', this._inst.token);
    setOrClearLocalStorage('idToken', this._inst.idToken);
    setOrClearLocalStorage('refreshToken', this._inst.refreshToken);
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

/**
 * Gets an item from localStorage, unlike `localStorage.getItem`, this returns undefined if item
 * does not exists.
 * @param key
 */
function getLocalStorage(key: string): string | undefined {
  const item = localStorage.getItem(key);
  return item ? item : undefined;
}

function setOrClearLocalStorage(key: string, value?: string): void {
  if (value === undefined) {
    localStorage.removeItem(key);
  } else {
    localStorage.setItem(key, value);
  }
}
