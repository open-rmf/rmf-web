import Big from 'big.js'
import { Knot } from "../util/cublic-spline"

export interface TrajectoryRequestParam {
  map_name: string
  start_time: string
  finish_time: string
}

export interface TrajectoryRequest {
  request: 'trajectory'
  param: TrajectoryRequestParam
}

export function trajectoryRequest(param: TrajectoryRequestParam): string {
  const requestObject: TrajectoryRequest = {
    request: 'trajectory',
    param,
  }
  return JSON.stringify(requestObject)
}

// RawVelocity received from server is in this format (x, y, theta)
export type RawVelocity = [number, number, number]

// RawPose2D received from server is in this format (x, y, theta)
export type RawPose2D = [number, number, number]

export interface RawKnot {
  t: string // nanoseconds
  v: RawVelocity
  x: RawPose2D
}

export interface TrajectoryResponseValue {
  dimensions: number[] 
  segments: RawKnot[] 
  shape: string
}

export interface TrajectoryResponse {
  response: 'trajectory'
  values: TrajectoryResponseValue[] | null
}

export function fromRawKnotsToKnots(rawKnots: RawKnot[]): Knot[] {
  const knots: Knot[] = []

  for (const rawKnot of rawKnots) {
    const [poseX, poseY, poseTheta] = rawKnot.x
    const [velocityX, velocityY, velocityTheta] = rawKnot.v
    knots.push({
      pose: {
        x: poseX,
        y: poseY,
        theta: poseTheta,
      },
      velocity: {
        x: velocityX,
        y: velocityY,
        theta: velocityTheta,
      },
      time: new Big(rawKnot.t),
    })
  }

  return knots
}