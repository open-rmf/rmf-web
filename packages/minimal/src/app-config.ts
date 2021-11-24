import { SioClient } from 'api-client';

if (!process.env.REACT_APP_RMF_SERVER) {
  throw new Error('REACT_APP_RMF_SERVER is required');
}

export const rmfServerUrl = process.env.REACT_APP_RMF_SERVER;
export const sioClient = new SioClient(rmfServerUrl);
