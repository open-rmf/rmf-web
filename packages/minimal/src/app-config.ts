import { SioClient, TasksApi, Configuration } from 'api-client';
import axios from 'axios';

const axiosInst = axios.create();
if (!process.env.REACT_APP_RMF_SERVER) {
  throw new Error('REACT_APP_RMF_SERVER is required');
}

const rmfServerUrl = process.env.REACT_APP_RMF_SERVER;
const apiConfig = new Configuration({
  basePath: rmfServerUrl,
});

export const currentLocation = process.env.REACT_APP_CURRENT_LOCATION;
export const sioClient = new SioClient(rmfServerUrl);
export const taskApi = new TasksApi(apiConfig, undefined, axiosInst);
