import '@testing-library/jest-dom';

const env = import.meta.env as any;
env.VITE_TRAJECTORY_SERVER = 'http://localhost';
env.VITE_AUTH_PROVIDER = '';
env.VITE_KEYCLOAK_CONFIG = '';
env.VITE_RMF_SERVER = 'http://localhost';
env.PUBLIC_URL = 'http://localhost';
