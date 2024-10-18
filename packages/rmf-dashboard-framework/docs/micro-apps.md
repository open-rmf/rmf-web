## MicroApps

"MicroApps" are the building blocks of `rmf-dashboard-framework`. Each "window" you see in the demo example is a MicroApp. Under the hood, MicroApps are really just react components that receive props of the type `MicroAppProps`.

### Creating a MicroApp

The easiest way to create a MicroApp is to use the `createMicroApp` function. The function takes a `appId`, `displayName`, a callback that returns a dynamic import and a callback that return the props passed to the component.

For example, if you have a `Foo` component in `foo.tsx`, this is how you can create a MicroApp from it.

foo.tsx

```tsx
export const Foo = () => <div>hello</div>;
export default Foo;
```

foo-app.tsx

```ts
createMicroApp(
  'example.foo',
  'Foo',
  () => import('./foo'),
  () => ({}),
);
```

Notice that we never import `Foo` directly, instead we only import it dynamically, this is so that vite can code-split the MicroApps and load only the MicroApps used in a tab.

### Setting Props in the MicroApp

The above example works because `Foo` does not take any props, but what if `Foo` display a message based on the props?

foo.tsx

```tsx
export const Foo = ({ message }) => <div>{message}</div>;
export default Foo;
```

`message` will always be undefined because `rmf-dashboard-framework` does not know how to render your component, infact it wouldn't even build because there will be typescript errors.

In this case, you can tell `rmf-dashboard-framework` how to render your component with the `props` param.

foo-app.ts

```ts
createMicroApp(
  'example.foo',
  'Foo',
  () => import('./foo'),
  () => ({ message: 'hello' }),
);
```

### MicroApp Settings

Your component may want to expose some settings for the user to change, you can do that with `rmf-dashboard-framework`'s `useSettings` and `useAppController` hooks.

foo.tsx

```tsx
export const Foo = () => {
  const settings = useSettings();
  const fooSettings = settings.microAppSettings['example.foo'];
  const { updateSettings } = useAppController(); // call `updateSettings` to save new settings.

  return <div>{fooSettings.message}</div>;
};

export default Foo;
```

An alternative is to use the `settings` and `updateSettings` params in the `props` callback.

foo-app.ts

```ts
createMicroApp(
  'example.foo',
  'Foo',
  () => import('./foo'),
  (settings, updateSettings) => ({ message: settings.microAppSettings['example.foo'].message }),
);
```

Using the hooks is the recommended approach as the gives you the most flexibility in how to manage the settings.

### Integrating with RMF

In most cases, you want your MicroApp to display some information from RMF, `rmf-dashboard-framework` provide hooks for you to do that. The most important hook is `useRmfApi`, this will return a `RmfApi` instance that you can use to interact with RMF.

For example, this will create a component that display the current floor of the lift `main_lift`.

foo.tsx

```tsx
export const Foo = () => {
  const [liftState, setLiftState] = React.useState<LiftState | null>(null);
  const rmfApi = useRmfApi();

  React.useEffect(() => {
    rmfApi.getLiftStateObs('main_lift').subscribe(setLiftState);
  }, [rmfApi]);

  return <div>{liftState?.current_floor || 'Unknown'}</div>;
};
```

### MicroApps with Custom Toolbar

When you create a MicroApp with `createMicroApp`, it will use a default toolbar, in some cases, you may want to expose some actions on the toolbar. The `Tasks` MicroApp is an example that uses the toolbar this way. Since "MicroApps" are really just react components that takes a subset of `Window` props, you can create a custom MicroApp with a custom toolbar by creating a similar component to `Window`.

foo.tsx

```tsx
export const Foo = ({ onClose, ...otherProps }: MicroAppProps) => {
  const [liftState, setLiftState] = React.useState<LiftState | null>(null);
  const rmfApi = useRmfApi();

  React.useEffect(() => {
    rmfApi.getLiftStateObs('main_lift').subscribe(setLiftState);
  }, [rmfApi]);

  return (
    <Window
      title="Foo"
      toolbar={
        <WindowToolbar title="Foo">
          <Button>Request Lift</Button>
          <WindowCloseButton onClick={() => onClose && onClose()} />
        </WindowToolbar>
      }
      {...otherProps}
    >
      <div>{liftState?.current_floor || 'Unknown'}</div>
    </Window>
  );
};
```

foo-app.tsx

```tsx
const Foo = lazy(() => import('./foo'));

export default {
  appId: 'example.foo',
  displayName: 'Foo',
  Component: (props) => (
    <Suspense fallback={null}>
      <Foo {...props} />
    </Suspense>
  ),
} satisfies MicroAppManifest;
```

Keep in mind that when creating a custom MicroApp this way, it is not enforced via typescript that the component is loaded lazily but it is still highly recommended for you to do so. Also unlike MicroApps created with `createMicroApp`, the entire `Window` is lazy loaded, not just the contents, this means that while the MicroApp is loading, nothing would be rendered unless you provide a `fallback` to `Suspense`.
