/// <reference types="react-scripts" />

declare namespace NodeJS {
  interface ProcessEnv {
    readonly REACT_APP_TRAJECTORY_SERVER?: string;
    readonly REACT_APP_AUTH_PROVIDER?: string;
    readonly REACT_APP_KEYCLOAK_CONFIG?: string;
    readonly REACT_APP_RMF_SERVER?: string; // socket.io server address, defaults to "<host>/socket.io"
    readonly PUBLIC_URL?: string;
    readonly BUILD_PATH?: string;
  }
}
