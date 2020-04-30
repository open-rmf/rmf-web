import { makeStyles } from '@material-ui/core';
import React from 'react';
import { Trajectory } from '../../robot-trajectory-manager';
import { RobotTrajectoryProps } from './robot-trajectory';

export interface TrajectoryAnimationProps extends React.SVGAttributes<SVGPathElement> {
  trajPath: TrajectoryPath;
  duration: number;
}

export interface TrajectoryPath {
  traj: Trajectory;
  d: string;
  segOffsets: number[];
}

/**
 * Fills the trajectory as the robot moves through it.
 * @param TrajectoryComponent
 * @param animationScale Animation duration is calculated from the trajectory and the animation
 * scale. For example, a trajectory that lasts 10s with an animation of scale of will play over
 * 10/5 = 2s.
 */
export function withFillAnimation(
  TrajectoryComponent: React.ComponentType<RobotTrajectoryProps>,
  animationScale: number,
): React.ComponentType<RobotTrajectoryProps> {
  return props => {
    const classes = useFillStyles();
    const { trajectory } = props;
    const pathRef = React.useRef<SVGPathElement>(null);

    React.useLayoutEffect(() => {
      if (!pathRef.current) {
        return;
      }

      const offsets = keyframeOffsets(trajectory);
      const pathAnim = pathRef.current.cloneNode(false) as SVGPathElement;
      pathAnim.classList.add(classes.anim);
      pathAnim.classList.remove(classes.highlight);
      pathAnim.setAttribute('path-length', '1');
      pathRef.current.parentElement?.appendChild(pathAnim);

      pathAnim.animate(
        offsets.map(offset => ({
          offset: offset,
          strokeDashoffset: 2 - offset,
        })),
        {
          duration: animationDuration(trajectory, animationScale),
          easing: 'linear',
          fill: 'forwards',
        },
      );

      return () => pathAnim.remove();
    }, [trajectory, classes.anim, classes.highlight]);

    return (
      <g>
        <TrajectoryComponent ref={pathRef} {...props} className={classes.highlight} />
      </g>
    );
  };
}

/**
 * Follows the robot's position as it moves through its trajectory.
 * @param TrajectoryComponent
 * @param animationScale Animation duration is calculated from the trajectory and the animation
 * scale. For example, a trajectory that lasts 10s with an animation of scale of will play over
 * 10/5 = 2s.
 */
export function withFollowAnimation(
  TrajectoryComponent: React.ComponentType<RobotTrajectoryProps>,
  animationScale: number,
): React.ComponentType<RobotTrajectoryProps> {
  return props => {
    const classes = useFollowStyles();
    const { trajectory } = props;
    const pathRef = React.useRef<SVGPathElement>(null);

    React.useLayoutEffect(() => {
      if (!pathRef.current) {
        return;
      }

      const offsets = keyframeOffsets(trajectory);
      const pathAnim = pathRef.current.cloneNode(false) as SVGPathElement;
      pathAnim.classList.add(classes.anim);
      pathAnim.classList.remove(classes.highlight);
      pathAnim.setAttribute('path-length', '1');
      const strokeWidth = Number(pathAnim.getAttribute('stroke-width') || 1);
      const strokeDash = strokeWidth / pathAnim.getTotalLength();
      pathAnim.setAttribute('stroke-dasharray', `${strokeDash} ${2 - strokeDash}`);
      pathRef.current.parentElement?.appendChild(pathAnim);

      pathAnim.animate(
        offsets.map(offset => ({
          offset: offset,
          strokeDashoffset: Math.max(2 - offset, strokeDash + 1),
        })),
        {
          duration: animationDuration(trajectory, animationScale),
          easing: 'linear',
          fill: 'forwards',
        },
      );

      return () => pathAnim.remove();
    }, [trajectory, classes.anim, classes.highlight]);

    return (
      <g>
        <TrajectoryComponent ref={pathRef} {...props} className={classes.highlight} />
      </g>
    );
  };
}

/**
 * Similar to fill animation, but only fills the outline of the trajectory.
 * @param TrajectoryComponent
 * @param animationScale Animation duration is calculated from the trajectory and the animation
 * scale. For example, a trajectory that lasts 10s with an animation of scale of will play over
 * 10/5 = 2s.
 */
export function withOutlineAnimation(
  TrajectoryComponent: React.ComponentType<RobotTrajectoryProps>,
  animationScale: number,
): React.ComponentType<RobotTrajectoryProps> {
  return props => {
    const classes = useOutlineStyles();
    const { trajectory } = props;
    const pathRef = React.useRef<SVGPathElement>(null);

    React.useLayoutEffect(() => {
      if (!pathRef.current || !pathRef.current.parentElement) {
        return;
      }

      const offsets = keyframeOffsets(trajectory);
      const parent = pathRef.current.parentElement;

      const mask = document.createElementNS('http://www.w3.org/2000/svg', 'mask');
      mask.setAttribute('id', `${trajectory.id}-mask`);
      const maskRect = document.createElementNS('http://www.w3.org/2000/svg', 'rect');
      maskRect.setAttribute('x', '0');
      maskRect.setAttribute('y', '0');
      maskRect.setAttribute('width', '100%');
      maskRect.setAttribute('height', '100%');
      maskRect.setAttribute('fill', 'white');
      mask.appendChild(maskRect);

      const maskPath = pathRef.current.cloneNode(false) as SVGPathElement;
      maskPath.classList.add(classes.maskPath);
      const pathStrokeWidth = pathRef.current.getAttribute('stroke-width');
      if (pathStrokeWidth) {
        maskPath.setAttribute('stroke-width', (parseFloat(pathStrokeWidth) - 0.1).toString());
      }
      mask.appendChild(maskPath);
      parent.appendChild(mask);

      const highlight = pathRef.current.cloneNode(false) as SVGPathElement;
      highlight.removeAttribute('mask');
      highlight.classList.add(classes.highlight);
      parent.appendChild(highlight);

      pathRef.current.animate(
        offsets.map(offset => ({
          offset: offset,
          strokeDashoffset: 2 - offset,
        })),
        {
          duration: animationDuration(trajectory, animationScale),
          easing: 'linear',
          fill: 'forwards',
        },
      );

      return () => {
        highlight.remove();
        mask.remove();
      };
    }, [trajectory, classes.highlight, classes.maskPath]);

    return (
      <g>
        <TrajectoryComponent ref={pathRef} {...props} mask={`url(#${trajectory.id}-mask)`} />
      </g>
    );
  };
}

function keyframeOffsets(traj: Trajectory): number[] {
  const { segments } = traj;
  const totalDuration = segments[segments.length - 1].t - segments[0].t;
  return traj.segments.map(seg => (seg.t - segments[0].t) / totalDuration);
}

const useFillStyles = makeStyles(() => ({
  anim: {
    opacity: 0.8,
    strokeDasharray: 2,
    strokeDashoffset: 2,
  },
  highlight: {
    opacity: 0.4,
  },
}));

const useFollowStyles = makeStyles(() => ({
  anim: {
    opacity: 0.8,
    strokeDashoffset: 2,
  },
  highlight: {
    opacity: 0.4,
  },
}));

const useOutlineStyles = makeStyles(() => ({
  highlight: {
    opacity: 0.25,
  },

  maskPath: {
    stroke: 'black',
    opacity: 1,
  },
}));

function animationDuration(traj: Trajectory, scale: number): number {
  return (traj.segments[traj.segments.length - 1].t - traj.segments[0].t) / scale;
}
