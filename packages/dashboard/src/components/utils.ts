import { AxiosError } from 'axios';

export function getApiErrorMessage(error: unknown): string {
  return (error as AxiosError).response?.data.detail || '';
}
