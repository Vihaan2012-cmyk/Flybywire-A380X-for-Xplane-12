A380_LOADING_SCREEN = true
do
  local dir = getAircraftPath() .. "/plugins/fbw_a380_systems/loading/"
  local ok, m = pcall(dofile, dir .. "manifest.lua")
  if ok and type(m) == "table" then
    local FADE_IN, MIN_VISIBLE, HOLD, FADE_OUT, TIMEOUT, SLIDE, CROSSFADE = 0.6, 10, 2, 1.5, 240, 8, 1.2
    local frp = globalPropertyf("sim/operation/misc/frame_rate_period")
    local tex = {}
    local function load(name)
      if tex[name] == nil then
        local id, w, h = sasl.gl.loadImage(dir .. name .. ".png")
        tex[name] = id and {id, w, h} or false
      end
      return tex[name]
    end
    for name, _ in pairs(m.sprites) do load(name) end
    load("white")
    load("bar_glow")
    local nbg, ntip = #m.backgrounds, #m.tips
    local seed = os.time()
    local first_bg = nbg > 0 and seed % nbg or 0
    local first_tip = ntip > 0 and math.floor(seed / 7) % ntip or 0
    local function bg_name(k) return ((m.backgrounds[(first_bg + k) % nbg + 1]):gsub("%.png$", "")) end
    local function tip_name(k) return m.tips[(first_tip + k) % ntip + 1] end
    local t, shown, complete_t, closing_t, finished = 0, 0, nil, nil, false

    local a, b, c, d = sasl.windows.getScreenBoundsGlobal()
    local x, y, w, h = a, b, c, d
    if c > a and d < b then x, y, w, h = a, d, c - a, b - d end
    local win = contextWindow {
      name = "A380 loading",
      position = {x, y, w, h},
      noDecore = true,
      noBackground = true,
      noResize = true,
      noMove = true,
      visible = true,
      layer = SASL_CW_LAYER_FLOATING_WINDOWS,
      components = {},
    }

    win.component.draw = function()
      if finished then return end
      local dt = math.min(get(frp) or 0, 0.25)
      t = t + dt
      local percent = rd("fbw/A380X_LOADING_PERCENT")
      local complete = rd("fbw/A380X_LOADING_COMPLETE") ~= 0
      if complete then
        complete_t = complete_t or t
        if t - complete_t > HOLD and t > MIN_VISIBLE then closing_t = closing_t or t end
      end
      if t > TIMEOUT then closing_t = closing_t or t end
      local closing = closing_t and (t - closing_t) / FADE_OUT or 0
      if closing >= 1 then
        finished = true
        for _, v in pairs(tex) do if v then sasl.gl.unloadTexture(v[1]) end end
        tex = {}
        win:setIsVisible(false)
        return
      end
      local alpha = math.min(t / FADE_IN, 1) * (1 - closing)
      shown = shown + (percent - shown) * math.min(dt * 4, 1)

      local slide = math.floor(t / SLIDE)
      local blend = math.max(0, math.min(1, (t - slide * SLIDE - (SLIDE - CROSSFADE)) / CROSSFADE))
      local white = tex["white"]
      if white then sasl.gl.drawTexture(white[1], 0, 0, w, h, 0.02, 0.03, 0.05, alpha) end
      if nbg > 0 then
        local current = load(bg_name(slide))
        local nxt = blend > 0 and load(bg_name(slide + 1)) or nil
        for _, layer in ipairs({{current, 1}, {nxt, blend}}) do
          local img, la = layer[1], layer[2]
          if img and la > 0 then
            local tw, th = img[2], img[3]
            local scale = math.max(w / tw, h / th)
            local cw, ch = w / scale, h / scale
            sasl.gl.drawTexturePart(img[1], 0, 0, w, h, (tw - cw) / 2, (th - ch) / 2, cw, ch, 1, 1, 1, alpha * la)
          end
        end
      end

      local s = w / m.design_width
      local lh = m.layer_height
      local function sprite(name, dx, sa)
        local r, img = m.sprites[name], tex[name]
        if r and img then
          sasl.gl.drawTexture(img[1], (r[1] + dx) * s, (lh - r[2] - r[4]) * s, r[3] * s, r[4] * s, 1, 1, 1, alpha * sa)
        end
      end
      local band = tex["band"]
      if band then sasl.gl.drawTexture(band[1], 0, 0, w, lh * s, 1, 1, 1, alpha) end
      sprite("wordmark", 0, 1)
      if ntip > 0 then
        sprite(tip_name(slide), 0, 1 - blend)
        sprite(tip_name(slide + 1), 0, blend)
      end
      sprite("loading", 0, 1)
      sprite("percent", 0, 1)
      local digits = tostring(math.floor(math.max(0, math.min(100, shown)) + 0.5))
      for i = #digits, 1, -1 do
        sprite("digit_" .. digits:sub(i, i), -(#digits - i) * m.digits.advance, 1)
      end
      local systems = rd("fbw/A380X_LOADING_SYSTEMS") ~= 0
      local up, total = rd("fbw/A380X_LOADING_SCREENS_UP"), rd("fbw/A380X_LOADING_SCREENS")
      local states = {
        systems and "done" or "active",
        (total > 0 and up >= total) and "done" or (systems and "active" or "waiting"),
        complete and "done" or "waiting",
      }
      for row, state in ipairs(states) do sprite("item_" .. (row - 1) .. "_" .. state, 0, 1) end
      if white then
        local bar = 6 * s
        local fill = w * math.max(0, math.min(1, shown / 100))
        sasl.gl.drawTexture(white[1], 0, 0, w, bar, 1, 1, 1, 0.12 * alpha)
        local glow = tex["bar_glow"]
        if glow then sasl.gl.drawTexture(glow[1], 0, 0, fill, 32 * s, 1, 1, 1, 0.8 * alpha) end
        sasl.gl.drawTexture(white[1], 0, 0, fill, bar, 0, 200 / 255, 1, alpha)
      end
    end
  else
    sasl.logWarning("A380 loading screen: no manifest at " .. dir)
  end
end
