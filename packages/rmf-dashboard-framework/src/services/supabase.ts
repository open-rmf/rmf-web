import {
  createClient,
  Provider,
  Session,
  SupabaseClient,
  SupabaseClientOptions,
} from '@supabase/supabase-js';
import Debug from 'debug';
import EventEmitter from 'eventemitter3';

import { Authenticator, AuthenticatorEventType } from './authenticator';

const debug = Debug('authenticator');

export interface SupabaseAuthenticatorConfig {
  supabaseUrl: string;
  supabaseAnonKey: string;
  /**
   * OAuth provider to use when `login()` is called. If omitted, `login()` falls
   * back to redirecting the user to a `/login` page (which the host app must
   * provide and call `signInWithPassword` on the underlying client).
   */
  oauthProvider?: Provider;
  /**
   * Forwarded to `createClient`. Use this to override storage, fetch, etc.
   */
  clientOptions?: SupabaseClientOptions<'public'>;
}

export class SupabaseAuthenticator
  extends EventEmitter<AuthenticatorEventType>
  implements Authenticator
{
  get user(): string | undefined {
    return this._user;
  }

  get token(): string | undefined {
    return this._session?.access_token;
  }

  get client(): SupabaseClient {
    return this._client;
  }

  constructor(config: SupabaseAuthenticatorConfig) {
    super();
    this._config = config;
    this._client = createClient(config.supabaseUrl, config.supabaseAnonKey, {
      auth: { autoRefreshToken: true, persistSession: true, detectSessionInUrl: true },
      ...config.clientOptions,
    });
  }

  async init(): Promise<void> {
    if (this._initialized) {
      debug('already initialized');
      return;
    }

    this._client.auth.onAuthStateChange((event, session) => {
      debug(`auth state change: ${event}`);
      this._setSession(session);
      if (event === 'TOKEN_REFRESHED') {
        this.emit('tokenRefresh', null);
      }
    });

    const { data, error } = await this._client.auth.getSession();
    if (error) {
      debug(`getSession error: ${error.message}`);
    }
    this._setSession(data.session ?? null);
    this._initialized = true;
  }

  async refreshToken(): Promise<void> {
    if (!this._initialized || !this._session) {
      return;
    }
    // Refresh only when within 30s of expiry; supabase-js's autoRefreshToken
    // handles background refresh, but RmfApi calls this before each request.
    const expiresAt = this._session.expires_at;
    if (expiresAt && expiresAt - Math.floor(Date.now() / 1000) > 30) {
      return;
    }
    const { data, error } = await this._client.auth.refreshSession();
    if (error) {
      debug(`refreshSession error: ${error.message}`);
      return;
    }
    this._setSession(data.session ?? null);
    this.emit('tokenRefresh', null);
  }

  async login(successRedirectUri?: string): Promise<never> {
    const redirectTo = successRedirectUri ?? window.location.href;

    if (this._config.oauthProvider) {
      await this._client.auth.signInWithOAuth({
        provider: this._config.oauthProvider,
        options: { redirectTo },
      });
      return new Promise<never>(() => {}); // navigation away
    }

    // Fallback: email + password via window.prompt(). Good enough for the
    // free-tier demo; production setups should pass `oauthProvider` or
    // render a real login form.
    //
    // We don't return Promise<never> here because the user stays on the
    // same page — but the Authenticator interface declares Promise<never>,
    // so we resolve with a sentinel rejection if the user cancels.
    const email = window.prompt('Supabase email');
    if (!email) {
      throw new Error('login cancelled');
    }
    const password = window.prompt('Supabase password');
    if (!password) {
      throw new Error('login cancelled');
    }

    const { data, error } = await this._client.auth.signInWithPassword({ email, password });
    if (error) {
      window.alert(`Login failed: ${error.message}`);
      throw error;
    }
    this._setSession(data.session ?? null);
    window.location.assign(redirectTo);
    return new Promise<never>(() => {});
  }

  async logout(): Promise<never> {
    await this._client.auth.signOut();
    window.location.assign('/');
    return new Promise<never>(() => {});
  }

  private _setSession(session: Session | null): void {
    this._session = session;
    const nextUser = session?.user
      ? // Prefer email for human-readability; falls back to UUID for OAuth
        // providers that don't expose email.
        session.user.email ?? session.user.id
      : undefined;
    if (nextUser !== this._user) {
      this._user = nextUser;
      this.emit('userChanged', this._user ?? null);
    }
  }

  private readonly _config: SupabaseAuthenticatorConfig;
  private readonly _client: SupabaseClient;
  private _initialized = false;
  private _session: Session | null = null;
  private _user?: string;
}

export default SupabaseAuthenticator;
