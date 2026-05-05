## Supabase project config for rmf-web

Code-defined Supabase setup for the rmf-web api-server. Stick to the bits declared here and the project will fit comfortably inside the Supabase free tier while running rmf_demos simulations.

### Files

- [`config.toml`](./config.toml) — auth providers, JWT settings, redirect
  URLs, and the access-token hook registration. Applied with
  `supabase config push`.
- [`migrations/`](./migrations) — versioned SQL applied by `supabase db push`.
  `20260505000000_rmf_auth_setup.sql` creates the `rmf_user_roles` table,
  the `rmf_access_token_hook` function (injects `app_metadata.roles` into
  JWTs), and the `rmf_grant_superuser(email)` helper. Idempotent.

### Initial setup

The Supabase CLI is pinned as a devDependency on this package, so you don't
need a global install.

> All commands below are run from the **root of `rmf-web/`**.

```bash
pnpm -C packages/api-server exec supabase login
pnpm -C packages/api-server exec supabase link --project-ref <your-project-ref>
pnpm -C packages/api-server exec supabase config push
pnpm -C packages/api-server exec supabase db push       # applies everything under migrations/
```

`supabase db push` runs the migrations directly against the linked cloud
project — no local `psql` install required. (The CLI prompts for the
database password the first time; it's the password you set when creating
the project.)

### Promote a user to admin

After the user has signed up at least once:

```sql
select public.rmf_grant_superuser('admin@example.com');
```

The next access token they receive will carry `app_metadata.roles =
["superuser"]`. The api-server's [authenticator.py](../api_server/authenticator.py)
reads that claim on first request and flips `User.is_admin` in its own DB.

### Free-tier notes

Confirmed against the free-tier limits as of 2026-05:

- Auth: unlimited users, 50k MAU cap. Email/password and OAuth providers
  are all free; Phone and SAML are paid.
- DB: 500 MB. rmf-web's hot tables (`LiftState`, `DoorState`,
  `DispenserState`, `IngestorState`) are upserts keyed on entity, so
  steady-state size is bounded by fleet count. Tasks and logs do grow —
  prune periodically if you run long demos.
- Project pauses after 7 days of inactivity. Run a heartbeat or just hit
  the dashboard occasionally during demo windows.
- Realtime / Storage: not used by rmf-web — disabled in `config.toml`.
