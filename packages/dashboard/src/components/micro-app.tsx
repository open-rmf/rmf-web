import React from 'react';
import { Window } from 'react-components';

export interface MicroAppProps {
  key: string;
  onClose?: () => void;
}

export function createMicroApp(
  title: string,
  Component: React.ComponentType<{}>,
): React.ComponentType<MicroAppProps> {
  return React.memo(
    React.forwardRef(
      (
        { children, ...otherProps }: React.PropsWithChildren<MicroAppProps>,
        ref: React.Ref<HTMLDivElement>,
      ) => (
        <Window ref={ref} title={title} {...otherProps}>
          <Component />
          {children}
        </Window>
      ),
    ),
  );
}
