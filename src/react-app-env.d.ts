/// <reference types="react-scripts" />
declare namespace NodeJS {
  interface ProcessEnv {
    readonly REACT_APP_MOCK: boolean;
    readonly REACT_APP_SOSS_NODE_NAME?: string;
    readonly REACT_APP_SOSS_SERVER: string;
    // TODO: remove this, use an auth service to get the token instead
    readonly REACT_APP_SOSS_TOKEN: string;
    readonly REACT_APP_TRAJECTORY_SERVER: string;
  }
}
