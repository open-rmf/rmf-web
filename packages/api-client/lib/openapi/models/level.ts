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
/**
 *
 * @export
 * @interface Level
 */
export interface Level {
  /**
   *
   * @type {string}
   * @memberof Level
   */
  name?: any;
  /**
   *
   * @type {number}
   * @memberof Level
   */
  elevation?: any;
  /**
   *
   * @type {Array&lt;AffineImage&gt;}
   * @memberof Level
   */
  images: any;
  /**
   *
   * @type {Array&lt;Place&gt;}
   * @memberof Level
   */
  places?: any;
  /**
   *
   * @type {Array&lt;Door&gt;}
   * @memberof Level
   */
  doors?: any;
  /**
   *
   * @type {Array&lt;Graph&gt;}
   * @memberof Level
   */
  nav_graphs?: any;
  /**
   *
   * @type {Graph}
   * @memberof Level
   */
  wall_graph?: any;
}
