/// <reference types="react-scripts" />

declare namespace NodeJS {
  interface ProcessEnv {
    readonly REACT_APP_MOCK?: boolean;
    readonly REACT_APP_SOSS_NODE_NAME?: string;
    readonly REACT_APP_SOSS_SERVER?: string;
    readonly REACT_APP_TRAJECTORY_SERVER?: string;
    readonly REACT_APP_AUTH_CONFIG?: string;
    readonly REACT_APP_API_SERVER?: string;
  }
}
