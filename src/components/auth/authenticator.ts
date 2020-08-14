import EventEmitter from 'eventemitter3';
import { UserManager } from 'oidc-client';
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
   * @param redirectUri
   */
  login(): Promise<never>;

  /**
   * Note: This redirects to external logout page so it will never return.
   */
  logout(): Promise<void>;
}

type SettingsType<T = typeof UserManager> = T extends new (settings: infer U) => unknown
  ? U
  : never;

export class DefaultAuthenticator extends EventEmitter<AuthenticatorEventType>
  implements Authenticator {
  get user(): User | undefined {
    return this._user || undefined;
  }

  get sossToken(): string | undefined {
    return this._sossToken;
  }

  constructor(settings: SettingsType) {
    super();
    this._userMgr = new UserManager(settings);
  }

  async init() {
    const user = await (async () => {
      try {
        return await this._userMgr.signinRedirectCallback();
      } catch (e) {
        console.error(e);
        return await this._userMgr.getUser();
      }
    })();
    if (user) {
      this._user = {
        username: user.profile.preferred_username || '',
        sossToken: user.id_token,
      };
      this._sossToken = user.id_token;
      console.log(this._sossToken);
      this.emit('userChanged', this._user);
    }
  }

  async login(): Promise<never> {
    await this._userMgr.signinRedirect();
    // oidc-client library does not immediately redirect, this timeout prevents the exception from
    // triggering under normal circumstances.
    await new Promise(res => setTimeout(res, 10000));
    throw new Error('should not reach here');
  }

  async logout(): Promise<never> {
    this._userMgr.removeUser();
    this._userMgr.signoutRedirect();
    await new Promise(res => setTimeout(res, 10000));
    throw new Error('should not reach here');
  }

  private _userMgr: UserManager;
  private _user: User | null = null;
  private _sossToken?: string;
}
