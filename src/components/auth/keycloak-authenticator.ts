import Keycloak_, { KeycloakInstance } from 'keycloak-js';
import Authenticator from './authenticator';

type ConfigType<T = typeof Keycloak_> = T extends (config: infer U) => unknown ? U : never;

export default class KeycloakAuthenticator implements Authenticator {
  static async create(config: ConfigType): Promise<KeycloakAuthenticator> {
    const inst = Keycloak_(config);
    const token = getLocalStorage('token');
    const idToken = getLocalStorage('idToken');
    const refreshToken = getLocalStorage('refreshToken');
    await inst.init({ token, idToken, refreshToken });
    try {
      await inst.updateToken(30);
    } catch {}
    console.log(inst.idToken);
    setOrClearLocalStorage('token', inst.token);
    setOrClearLocalStorage('idToken', inst.idToken);
    setOrClearLocalStorage('refreshToken', inst.refreshToken);
    return new KeycloakAuthenticator(inst);
  }

  get authenticated(): boolean {
    return !!this._inst.authenticated;
  }

  get sossToken(): string | undefined {
    return this._inst.idToken;
  }

  async login(redirectUri?: string): Promise<never> {
    await this._inst.login({ redirectUri });
    throw new Error('should not reach here');
  }

  async logout(): Promise<never> {
    localStorage.removeItem('token');
    localStorage.removeItem('idToken');
    localStorage.removeItem('refreshToken');
    this._inst.logout();
    throw new Error('should not reach here');
  }

  private constructor(private _inst: KeycloakInstance) {}
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
