/**
 * WIP
 */
export interface AuthResult {
  success: boolean;
  err: string | undefined;
}

function arrayBufferToBase64(buffer: ArrayBuffer) {
  var binary = '';
  var bytes = new Uint8Array(buffer);
  var len = bytes.byteLength;
  for (var i = 0; i < len; i++) {
    binary += String.fromCharCode(bytes[i]);
  }
  return window.btoa(binary);
}

export interface AuthService {
  login(user: string, password: string): Promise<AuthResult>;
  token(): string;
  verifyToken(): Promise<boolean>;
  isAuthenticated(): boolean;
}

export class OldAuthService {
  constructor(private _url: string) {}

  async login(user: string, password: string): Promise<AuthResult> {
    console.log(`authentication to ${this._url}`);

    let encoder = new TextEncoder();
    let hashedPassword = await crypto.subtle.digest('SHA-256', encoder.encode(password));
    let b64 = arrayBufferToBase64(hashedPassword);

    const result: AuthResult = {
      success: false,
      err: '',
    };

    try {
      const resp = await fetch(`${this._url}/api/login`, {
        method: 'POST',
        body: JSON.stringify({ username: user, password: b64 }),
      });
      if (resp.ok) {
        const body = (await resp.json()) as AuthResp;
        this._validToken = true;
        this._saveToken(body.token);
        result.success = true;
      } else if (resp.status === 401) {
        result.success = false;
        result.err = 'Wrong username or password';
      } else {
        result.success = false;
        result.err = 'Unknown error';
      }
    } catch (e) {
      console.error((e as Error).message);
      result.success = false;
      result.err = (e as Error).message;
    }
    return result;
  }

  token(): string {
    const token = localStorage.getItem('token');
    return token ? token : '';
  }

  async verifyToken(): Promise<boolean> {
    if (!localStorage.getItem('token')) {
      this._validToken = false;
      return false;
    }
    try {
      const resp = await fetch(`${this._url}/api/verifyToken`, {
        method: 'POST',
        headers: {
          Authorization: `Bearer ${localStorage.getItem('token')}`,
        },
      });
      const body = (await resp.json()) as VerifyTokenResult;
      if (body.result) {
        this._validToken = true;
        return true;
      } else {
        this._validToken = false;
        return false;
      }
    } catch (e) {
      this._validToken = false;
      return false;
    }
  }

  isAuthenticated(): boolean {
    return this._validToken;
  }

  private _validToken = false;

  private _saveToken(token: string) {
    localStorage.setItem('token', token);
  }
}

interface AuthResp {
  result: string;
  token: string;
}

interface VerifyTokenResult {
  result: string;
}
