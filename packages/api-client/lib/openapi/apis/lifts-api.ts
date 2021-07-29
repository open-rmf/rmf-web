/* tslint:disable */
/* eslint-disable */
/**
 * RMF API Server
 * No description provided (generated by Swagger Codegen https://github.com/swagger-api/swagger-codegen)
 *
 * OpenAPI spec version: 0.1.0
 *
 *
 * NOTE: This class is auto generated by the swagger code generator program.
 * https://github.com/swagger-api/swagger-codegen.git
 * Do not edit the class manually.
 */
import globalAxios, { AxiosPromise, AxiosInstance } from 'axios';
import { Configuration } from '../configuration';
// Some imports not used depending on template conditions
// @ts-ignore
import { BASE_PATH, COLLECTION_FORMATS, RequestArgs, BaseAPI, RequiredError } from '../base';
import { ApiServerModelsTortoiseModelsHealthLiftHealthLeaf } from '../models';
import { HTTPValidationError } from '../models';
import { Lift } from '../models';
import { LiftRequest } from '../models';
import { LiftState } from '../models';
import { ModelObject } from '../models';
/**
 * LiftsApi - axios parameter creator
 * @export
 */
export const LiftsApiAxiosParamCreator = function (configuration?: Configuration) {
  return {
    /**
     * Available in socket.io
     * @summary Get Lift Health
     * @param {string} lift_name
     * @param {*} [options] Override http request option.
     * @throws {RequiredError}
     */
    getLiftHealthLiftsLiftNameHealthGet: async (
      lift_name: string,
      options: any = {},
    ): Promise<RequestArgs> => {
      // verify required parameter 'lift_name' is not null or undefined
      if (lift_name === null || lift_name === undefined) {
        throw new RequiredError(
          'lift_name',
          'Required parameter lift_name was null or undefined when calling getLiftHealthLiftsLiftNameHealthGet.',
        );
      }
      const localVarPath = `/lifts/{lift_name}/health`.replace(
        `{${'lift_name'}}`,
        encodeURIComponent(String(lift_name)),
      );
      // use dummy base URL string because the URL constructor only accepts absolute URLs.
      const localVarUrlObj = new URL(localVarPath, 'https://example.com');
      let baseOptions;
      if (configuration) {
        baseOptions = configuration.baseOptions;
      }
      const localVarRequestOptions = { method: 'GET', ...baseOptions, ...options };
      const localVarHeaderParameter = {} as any;
      const localVarQueryParameter = {} as any;

      const query = new URLSearchParams(localVarUrlObj.search);
      for (const key in localVarQueryParameter) {
        query.set(key, localVarQueryParameter[key]);
      }
      for (const key in options.query) {
        query.set(key, options.query[key]);
      }
      localVarUrlObj.search = new URLSearchParams(query).toString();
      let headersFromBaseOptions = baseOptions && baseOptions.headers ? baseOptions.headers : {};
      localVarRequestOptions.headers = {
        ...localVarHeaderParameter,
        ...headersFromBaseOptions,
        ...options.headers,
      };

      return {
        url: localVarUrlObj.pathname + localVarUrlObj.search + localVarUrlObj.hash,
        options: localVarRequestOptions,
      };
    },
    /**
     * Available in socket.io
     * @summary Get Lift State
     * @param {string} lift_name
     * @param {*} [options] Override http request option.
     * @throws {RequiredError}
     */
    getLiftStateLiftsLiftNameStateGet: async (
      lift_name: string,
      options: any = {},
    ): Promise<RequestArgs> => {
      // verify required parameter 'lift_name' is not null or undefined
      if (lift_name === null || lift_name === undefined) {
        throw new RequiredError(
          'lift_name',
          'Required parameter lift_name was null or undefined when calling getLiftStateLiftsLiftNameStateGet.',
        );
      }
      const localVarPath = `/lifts/{lift_name}/state`.replace(
        `{${'lift_name'}}`,
        encodeURIComponent(String(lift_name)),
      );
      // use dummy base URL string because the URL constructor only accepts absolute URLs.
      const localVarUrlObj = new URL(localVarPath, 'https://example.com');
      let baseOptions;
      if (configuration) {
        baseOptions = configuration.baseOptions;
      }
      const localVarRequestOptions = { method: 'GET', ...baseOptions, ...options };
      const localVarHeaderParameter = {} as any;
      const localVarQueryParameter = {} as any;

      const query = new URLSearchParams(localVarUrlObj.search);
      for (const key in localVarQueryParameter) {
        query.set(key, localVarQueryParameter[key]);
      }
      for (const key in options.query) {
        query.set(key, options.query[key]);
      }
      localVarUrlObj.search = new URLSearchParams(query).toString();
      let headersFromBaseOptions = baseOptions && baseOptions.headers ? baseOptions.headers : {};
      localVarRequestOptions.headers = {
        ...localVarHeaderParameter,
        ...headersFromBaseOptions,
        ...options.headers,
      };

      return {
        url: localVarUrlObj.pathname + localVarUrlObj.search + localVarUrlObj.hash,
        options: localVarRequestOptions,
      };
    },
    /**
     *
     * @summary Get Lifts
     * @param {*} [options] Override http request option.
     * @throws {RequiredError}
     */
    getLiftsLiftsGet: async (options: any = {}): Promise<RequestArgs> => {
      const localVarPath = `/lifts`;
      // use dummy base URL string because the URL constructor only accepts absolute URLs.
      const localVarUrlObj = new URL(localVarPath, 'https://example.com');
      let baseOptions;
      if (configuration) {
        baseOptions = configuration.baseOptions;
      }
      const localVarRequestOptions = { method: 'GET', ...baseOptions, ...options };
      const localVarHeaderParameter = {} as any;
      const localVarQueryParameter = {} as any;

      const query = new URLSearchParams(localVarUrlObj.search);
      for (const key in localVarQueryParameter) {
        query.set(key, localVarQueryParameter[key]);
      }
      for (const key in options.query) {
        query.set(key, options.query[key]);
      }
      localVarUrlObj.search = new URLSearchParams(query).toString();
      let headersFromBaseOptions = baseOptions && baseOptions.headers ? baseOptions.headers : {};
      localVarRequestOptions.headers = {
        ...localVarHeaderParameter,
        ...headersFromBaseOptions,
        ...options.headers,
      };

      return {
        url: localVarUrlObj.pathname + localVarUrlObj.search + localVarUrlObj.hash,
        options: localVarRequestOptions,
      };
    },
    /**
     *
     * @summary  Post Lift Request
     * @param {LiftRequest} body
     * @param {string} lift_name
     * @param {*} [options] Override http request option.
     * @throws {RequiredError}
     */
    postLiftRequestLiftsLiftNameRequestPost: async (
      body: LiftRequest,
      lift_name: string,
      options: any = {},
    ): Promise<RequestArgs> => {
      // verify required parameter 'body' is not null or undefined
      if (body === null || body === undefined) {
        throw new RequiredError(
          'body',
          'Required parameter body was null or undefined when calling postLiftRequestLiftsLiftNameRequestPost.',
        );
      }
      // verify required parameter 'lift_name' is not null or undefined
      if (lift_name === null || lift_name === undefined) {
        throw new RequiredError(
          'lift_name',
          'Required parameter lift_name was null or undefined when calling postLiftRequestLiftsLiftNameRequestPost.',
        );
      }
      const localVarPath = `/lifts/{lift_name}/request`.replace(
        `{${'lift_name'}}`,
        encodeURIComponent(String(lift_name)),
      );
      // use dummy base URL string because the URL constructor only accepts absolute URLs.
      const localVarUrlObj = new URL(localVarPath, 'https://example.com');
      let baseOptions;
      if (configuration) {
        baseOptions = configuration.baseOptions;
      }
      const localVarRequestOptions = { method: 'POST', ...baseOptions, ...options };
      const localVarHeaderParameter = {} as any;
      const localVarQueryParameter = {} as any;

      localVarHeaderParameter['Content-Type'] = 'application/json';

      const query = new URLSearchParams(localVarUrlObj.search);
      for (const key in localVarQueryParameter) {
        query.set(key, localVarQueryParameter[key]);
      }
      for (const key in options.query) {
        query.set(key, options.query[key]);
      }
      localVarUrlObj.search = new URLSearchParams(query).toString();
      let headersFromBaseOptions = baseOptions && baseOptions.headers ? baseOptions.headers : {};
      localVarRequestOptions.headers = {
        ...localVarHeaderParameter,
        ...headersFromBaseOptions,
        ...options.headers,
      };
      const needsSerialization =
        typeof body !== 'string' ||
        localVarRequestOptions.headers['Content-Type'] === 'application/json';
      localVarRequestOptions.data = needsSerialization
        ? JSON.stringify(body !== undefined ? body : {})
        : body || '';

      return {
        url: localVarUrlObj.pathname + localVarUrlObj.search + localVarUrlObj.hash,
        options: localVarRequestOptions,
      };
    },
  };
};

/**
 * LiftsApi - functional programming interface
 * @export
 */
export const LiftsApiFp = function (configuration?: Configuration) {
  return {
    /**
     * Available in socket.io
     * @summary Get Lift Health
     * @param {string} lift_name
     * @param {*} [options] Override http request option.
     * @throws {RequiredError}
     */
    async getLiftHealthLiftsLiftNameHealthGet(
      lift_name: string,
      options?: any,
    ): Promise<
      (
        axios?: AxiosInstance,
        basePath?: string,
      ) => AxiosPromise<ApiServerModelsTortoiseModelsHealthLiftHealthLeaf>
    > {
      const localVarAxiosArgs = await LiftsApiAxiosParamCreator(
        configuration,
      ).getLiftHealthLiftsLiftNameHealthGet(lift_name, options);
      return (axios: AxiosInstance = globalAxios, basePath: string = BASE_PATH) => {
        const axiosRequestArgs = {
          ...localVarAxiosArgs.options,
          url: basePath + localVarAxiosArgs.url,
        };
        return axios.request(axiosRequestArgs);
      };
    },
    /**
     * Available in socket.io
     * @summary Get Lift State
     * @param {string} lift_name
     * @param {*} [options] Override http request option.
     * @throws {RequiredError}
     */
    async getLiftStateLiftsLiftNameStateGet(
      lift_name: string,
      options?: any,
    ): Promise<(axios?: AxiosInstance, basePath?: string) => AxiosPromise<LiftState>> {
      const localVarAxiosArgs = await LiftsApiAxiosParamCreator(
        configuration,
      ).getLiftStateLiftsLiftNameStateGet(lift_name, options);
      return (axios: AxiosInstance = globalAxios, basePath: string = BASE_PATH) => {
        const axiosRequestArgs = {
          ...localVarAxiosArgs.options,
          url: basePath + localVarAxiosArgs.url,
        };
        return axios.request(axiosRequestArgs);
      };
    },
    /**
     *
     * @summary Get Lifts
     * @param {*} [options] Override http request option.
     * @throws {RequiredError}
     */
    async getLiftsLiftsGet(
      options?: any,
    ): Promise<(axios?: AxiosInstance, basePath?: string) => AxiosPromise<Array<Lift>>> {
      const localVarAxiosArgs = await LiftsApiAxiosParamCreator(configuration).getLiftsLiftsGet(
        options,
      );
      return (axios: AxiosInstance = globalAxios, basePath: string = BASE_PATH) => {
        const axiosRequestArgs = {
          ...localVarAxiosArgs.options,
          url: basePath + localVarAxiosArgs.url,
        };
        return axios.request(axiosRequestArgs);
      };
    },
    /**
     *
     * @summary  Post Lift Request
     * @param {LiftRequest} body
     * @param {string} lift_name
     * @param {*} [options] Override http request option.
     * @throws {RequiredError}
     */
    async postLiftRequestLiftsLiftNameRequestPost(
      body: LiftRequest,
      lift_name: string,
      options?: any,
    ): Promise<(axios?: AxiosInstance, basePath?: string) => AxiosPromise<ModelObject>> {
      const localVarAxiosArgs = await LiftsApiAxiosParamCreator(
        configuration,
      ).postLiftRequestLiftsLiftNameRequestPost(body, lift_name, options);
      return (axios: AxiosInstance = globalAxios, basePath: string = BASE_PATH) => {
        const axiosRequestArgs = {
          ...localVarAxiosArgs.options,
          url: basePath + localVarAxiosArgs.url,
        };
        return axios.request(axiosRequestArgs);
      };
    },
  };
};

/**
 * LiftsApi - factory interface
 * @export
 */
export const LiftsApiFactory = function (
  configuration?: Configuration,
  basePath?: string,
  axios?: AxiosInstance,
) {
  return {
    /**
     * Available in socket.io
     * @summary Get Lift Health
     * @param {string} lift_name
     * @param {*} [options] Override http request option.
     * @throws {RequiredError}
     */
    getLiftHealthLiftsLiftNameHealthGet(
      lift_name: string,
      options?: any,
    ): AxiosPromise<ApiServerModelsTortoiseModelsHealthLiftHealthLeaf> {
      return LiftsApiFp(configuration)
        .getLiftHealthLiftsLiftNameHealthGet(lift_name, options)
        .then((request) => request(axios, basePath));
    },
    /**
     * Available in socket.io
     * @summary Get Lift State
     * @param {string} lift_name
     * @param {*} [options] Override http request option.
     * @throws {RequiredError}
     */
    getLiftStateLiftsLiftNameStateGet(lift_name: string, options?: any): AxiosPromise<LiftState> {
      return LiftsApiFp(configuration)
        .getLiftStateLiftsLiftNameStateGet(lift_name, options)
        .then((request) => request(axios, basePath));
    },
    /**
     *
     * @summary Get Lifts
     * @param {*} [options] Override http request option.
     * @throws {RequiredError}
     */
    getLiftsLiftsGet(options?: any): AxiosPromise<Array<Lift>> {
      return LiftsApiFp(configuration)
        .getLiftsLiftsGet(options)
        .then((request) => request(axios, basePath));
    },
    /**
     *
     * @summary  Post Lift Request
     * @param {LiftRequest} body
     * @param {string} lift_name
     * @param {*} [options] Override http request option.
     * @throws {RequiredError}
     */
    postLiftRequestLiftsLiftNameRequestPost(
      body: LiftRequest,
      lift_name: string,
      options?: any,
    ): AxiosPromise<ModelObject> {
      return LiftsApiFp(configuration)
        .postLiftRequestLiftsLiftNameRequestPost(body, lift_name, options)
        .then((request) => request(axios, basePath));
    },
  };
};

/**
 * LiftsApi - object-oriented interface
 * @export
 * @class LiftsApi
 * @extends {BaseAPI}
 */
export class LiftsApi extends BaseAPI {
  /**
   * Available in socket.io
   * @summary Get Lift Health
   * @param {string} lift_name
   * @param {*} [options] Override http request option.
   * @throws {RequiredError}
   * @memberof LiftsApi
   */
  public getLiftHealthLiftsLiftNameHealthGet(lift_name: string, options?: any) {
    return LiftsApiFp(this.configuration)
      .getLiftHealthLiftsLiftNameHealthGet(lift_name, options)
      .then((request) => request(this.axios, this.basePath));
  }
  /**
   * Available in socket.io
   * @summary Get Lift State
   * @param {string} lift_name
   * @param {*} [options] Override http request option.
   * @throws {RequiredError}
   * @memberof LiftsApi
   */
  public getLiftStateLiftsLiftNameStateGet(lift_name: string, options?: any) {
    return LiftsApiFp(this.configuration)
      .getLiftStateLiftsLiftNameStateGet(lift_name, options)
      .then((request) => request(this.axios, this.basePath));
  }
  /**
   *
   * @summary Get Lifts
   * @param {*} [options] Override http request option.
   * @throws {RequiredError}
   * @memberof LiftsApi
   */
  public getLiftsLiftsGet(options?: any) {
    return LiftsApiFp(this.configuration)
      .getLiftsLiftsGet(options)
      .then((request) => request(this.axios, this.basePath));
  }
  /**
   *
   * @summary  Post Lift Request
   * @param {LiftRequest} body
   * @param {string} lift_name
   * @param {*} [options] Override http request option.
   * @throws {RequiredError}
   * @memberof LiftsApi
   */
  public postLiftRequestLiftsLiftNameRequestPost(
    body: LiftRequest,
    lift_name: string,
    options?: any,
  ) {
    return LiftsApiFp(this.configuration)
      .postLiftRequestLiftsLiftNameRequestPost(body, lift_name, options)
      .then((request) => request(this.axios, this.basePath));
  }
}
