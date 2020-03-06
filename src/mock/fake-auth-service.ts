import { AuthService, AuthResult } from '../auth-service';

export class FakeAuthService extends AuthService {
  async login(user: string, password: string): Promise<AuthResult> {
    return {
      success: true,
      err: '',
    };
  }

  token(): string {
    // { user: 'romi-dashboard' } signed with HS256 + secret 'rmf'
    return 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJ1c2VyIjoicm9taS1kYXNoYm9hcmQiLCJpYXQiOjE1ODMyODYyMTV9.x9aNjcLujQPHchWEsbrRbvctmnGQtEzw-81X0aPIE-Y';
  }

  async verifyToken(): Promise<boolean> {
    return true;
  }

  isAuthenticated(): boolean {
    return true;
  }
}
