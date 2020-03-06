interface AppConfig {
  sossUrl: string,
  authUrl: string,
}

export const appConfig: AppConfig = {
  sossUrl: 'wss://localhost:50001',
  authUrl: '',
};

export default appConfig;
