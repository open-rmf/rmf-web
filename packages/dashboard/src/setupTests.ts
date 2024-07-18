import '@testing-library/jest-dom';

const env = process.env as any;
env.REACT_APP_TRAJECTORY_SERVER = 'http://localhost';
env.REACT_APP_AUTH_PROVIDER = '';
env.REACT_APP_KEYCLOAK_CONFIG = '';
env.REACT_APP_RMF_SERVER = 'http://localhost';
env.PUBLIC_URL = 'http://localhost';
