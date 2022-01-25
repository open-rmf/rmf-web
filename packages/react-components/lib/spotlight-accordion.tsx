import { AccordionProps } from '@mui/material';
import React, { MutableRefObject } from 'react';

type ManagedProps = 'onChange' | 'expanded';

export interface SpotlightHandle {
  spotlight(): void;
}

export type SpotlightAccordionProps<P extends OptionalChildren<AccordionProps>> = Omit<
  P,
  ManagedProps
>;

type OptionalChildren<T extends React.PropsWithChildren<unknown>> = Omit<T, 'children'> & {
  children?: T['children'];
};

/**
 * Given an Accordion component, add support for putting it on "spotlight". This customize the
 * component so that it returns a ref with a `spotlight` method. Calling that method will scroll
 * the component into view and expand it.
 *
 * This overrides the `onChange` and `expanded` props so those are removed from the public interface.
 * @param BaseAccordion
 */
export function withSpotlight<P extends OptionalChildren<AccordionProps>>(
  BaseAccordion: React.ComponentType<P>,
): React.ForwardRefExoticComponent<
  React.PropsWithoutRef<SpotlightAccordionProps<P>> & React.RefAttributes<SpotlightHandle>
> {
  return React.forwardRef((props: SpotlightAccordionProps<P>, ref: React.Ref<SpotlightHandle>) => {
    const [expanded, setExpanded] = React.useState(false);
    const innerRef = React.useRef<HTMLElement>();

    React.useImperativeHandle(ref, () => {
      return {
        spotlight: () => {
          setExpanded(true);
          innerRef.current?.scrollIntoView({ behavior: 'smooth' });
        },
      };
    });

    return (
      <BaseAccordion
        expanded={expanded}
        onChange={(_, newExpanded) => setExpanded(newExpanded)}
        {...(props as P)}
      >
        {props.children}
      </BaseAccordion>
    );
  });
}

export interface SpotlightRef {
  ref: React.RefCallback<SpotlightHandle>;
  spotlight: () => void;
}

/**
 * Allows a spotlight to be called even when the component is not mounted. The spotlight will be
 * deferred until the component is mounted.
 */
export function createSpotlightRef(): SpotlightRef {
  const ref: MutableRefObject<SpotlightHandle | null> = {
    current: null,
  };
  const spotlightDefer: MutableRefObject<boolean> = {
    current: false,
  };

  const doSpotlight = () => {
    if (ref.current) {
      ref.current.spotlight();
    } else {
      spotlightDefer.current = true;
    }
  };

  const refCb: React.RefCallback<SpotlightHandle> = (newRef) => {
    if (spotlightDefer.current) {
      newRef?.spotlight();
      spotlightDefer.current = false;
    }
    ref.current = newRef;
  };

  return { ref: refCb, spotlight: doSpotlight };
}

export function useSpotlightRef(): React.MutableRefObject<SpotlightRef> {
  return React.useRef(createSpotlightRef());
}
