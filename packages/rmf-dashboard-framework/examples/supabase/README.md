## Supabase example

End-to-end demo of running the rmf-web stack against a **cloud** Supabase
project for authentication. Free-tier compatible.

This example covers the auth side only; the api-server still uses its own
local sqlite DB unless you also set `SUPABASE_DB_URL` (see step 3).

> **All commands below run from the root of `rmf-web/`.** Paths are written
> in full so it's unambiguous which file each command touches.

---

### 1. Provision the Supabase project

The Supabase CLI is pinned as a devDependency on `packages/api-server`, so
no global install is needed.

```bash
pnpm -C packages/api-server exec supabase login
pnpm -C packages/api-server exec supabase link --project-ref <your-project-ref>
pnpm -C packages/api-server exec supabase config push   # applies packages/api-server/supabase/config.toml
pnpm -C packages/api-server exec supabase db push       # applies packages/api-server/supabase/migrations/
```

---

### 2. Create the .env file

There is **one** `.env` file shared by both the dashboard and the api-server.

```bash
cp packages/rmf-dashboard-framework/examples/supabase/.env.example \
   packages/rmf-dashboard-framework/examples/supabase/.env
```

Open
[`packages/rmf-dashboard-framework/examples/supabase/.env`](./.env.example)
in your editor and fill in **all three required values**:

| Variable                 | Where to find it (Supabase dashboard → your project → Settings → API) |
| ------------------------ | --------------------------------------------------------------------- |
| `VITE_SUPABASE_URL`      | "Project URL" (e.g. `https://abcd1234.supabase.co`)                   |
| `VITE_SUPABASE_ANON_KEY` | "Project API keys" → **anon / public** key (not service_role)         |
| `SUPABASE_PROJECT_URL`   | Same value as `VITE_SUPABASE_URL`                                     |

`SUPABASE_JWT_SECRET` and `SUPABASE_DB_URL` stay commented out by default.
The api-server fetches the project's JWT signing keys from the JWKS endpoint
at `<SUPABASE_PROJECT_URL>/auth/v1/.well-known/jwks.json`, which is the
default for any Supabase project created since late 2025. Only set
`SUPABASE_JWT_SECRET` if your project's "JWT Settings" page shows a legacy
HS256 shared secret.

---

### 3. Bootstrap an admin user

a. Sign up a user via the Supabase dashboard
   (Authentication → Users → "Add user"), or via the dashboard once it's
   running in step 4.

b. In the Supabase dashboard's SQL editor, promote that user to admin:

```sql
select public.rmf_grant_superuser('you@example.com');
```

---

### 4. Run the dashboard

```bash
pnpm -C packages/rmf-dashboard-framework start:example examples/supabase
```

The `examples/supabase` argument is required — it tells vite which example
to serve. Open <http://localhost:5173/>.

Click **Login**. Two `window.prompt()` dialogs ask for your Supabase email
and password (the user you bootstrapped in step 3). This minimal flow is
intentional — the example doesn't ship a real login form so the demo stays
free-tier-only with no OAuth-provider setup. For a polished OAuth login,
pass `oauthProvider: 'github'` (or any provider you've enabled in
`config.toml`) when constructing `SupabaseAuthenticator` in
[`main.tsx`](./main.tsx).

---

### 5. Run the api-server

In a **second terminal** (also at the `rmf-web/` root):

```bash
source ~/workspaces/web-update/ws/install/setup.bash
packages/rmf-dashboard-framework/examples/supabase/run-api-server.bash
```

The script sources the `.env` file from step 2 and starts the api-server on
<http://localhost:8000>.

If you get
`SUPABASE_PROJECT_URL: must be set (see examples/supabase/.env.example)`,
it means `.env` doesn't exist or doesn't contain that variable — re-check
step 2 and confirm both api-server keys are filled in (not just the
`VITE_*` ones).

---

The dashboard at <http://localhost:5173> should now sign you in via
Supabase and talk to the api-server at <http://localhost:8000>.
