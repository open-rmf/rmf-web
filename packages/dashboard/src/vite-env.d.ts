/// <reference types="vite/client" />

interface ImportMetaEnv {
  readonly VITE_TRAJECTORY_SERVER?: string;
  readonly VITE_AUTH_PROVIDER?: string;
  readonly VITE_KEYCLOAK_CONFIG?: string;
  readonly VITE_RMF_SERVER?: string;
}

interface ImportMeta {
  readonly env: ImportMetaEnv;
}
