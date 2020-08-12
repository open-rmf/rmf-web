export default interface Authenticator {
  readonly authenticated: boolean;
  readonly sossToken?: string;

  /**
   * Note: This redirects to external login page so it will never return.
   * @param options
   */
  login(redirectUri?: string): Promise<never>;

  /**
   * Note: This redirects to external logout page so it will never return.
   * @param options
   */
  logout(): Promise<void>;
}
