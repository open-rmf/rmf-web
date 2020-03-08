import * as RomiCore from '@osrf/romi-js-core-interfaces';
import { SossTransport } from '@osrf/romi-js-soss-transport';
import { AuthService } from './auth-service';
import { FakeAuthService } from './mock/fake-auth-service';
import { FakeTransport } from './mock/fake-transport';

interface AppConfig {
  transportFactory: () => Promise<RomiCore.Transport>;
  main: () => Promise<any>;
}

let auth: AuthService;
export let appConfig: AppConfig;

if (!process.env.REACT_APP_MOCK) {
  auth = new FakeAuthService(); // TODO
  appConfig = {
    transportFactory: () =>
      SossTransport.connect('romi-dashboard', 'wss://localhost:50001', auth.token()),
    main: () => import('./main'),
  };
} else {
  auth = new FakeAuthService();
  appConfig = {
    transportFactory: async () => new FakeTransport(),
    main: () => import('./mock/index'),
  };
}

export default appConfig;
