# Layout API Reference

CppPlot provides flexible layout capabilities similar to Julia's Plots package and matplotlib's GridSpec.

## Quick Comparison

| Feature | CppPlot | Julia Plots | Matplotlib |
|---------|---------|-------------|------------|
| Basic subplot | `subplot(2,3,1)` | `plot(..., layout=(2,3))` | `plt.subplot(2,3,1)` |
| Span cells | `subplot(2,3,{1,2})` | `@layout [a{0.5w} b; c d e]` | `ax.subplot2grid((2,3),(0,0),colspan=2)` |
| Custom widths | `layout(2,3,{1,2,1})` | `grid(2,3,widths=[0.25,0.5,0.25])` | `GridSpec(2,3,width_ratios=[1,2,1])` |
| Free position | `add_axes(x,y,w,h)` | N/A | `fig.add_axes([x,y,w,h])` |
| Inset | `inset_axes(x,y,w,h)` | `inset=(1,(0.1,0.5,0.4,0.4))` | `inset_axes(ax,...)` |

## Basic Subplot Functions

### `subplot(nrows, ncols, index)`

Create or select a subplot in a regular grid.

```cpp
figure(800, 600);

subplot(2, 2, 1);  // Top-left
plot(x, y1);

subplot(2, 2, 2);  // Top-right
plot(x, y2);

subplot(2, 2, 3);  // Bottom-left
plot(x, y3);

subplot(2, 2, 4);  // Bottom-right
plot(x, y4);
```

**Parameters:**
- `nrows`: Number of rows in the grid
- `ncols`: Number of columns in the grid
- `index`: 1-based index (row-major order)

### `subplot(nrows, ncols, {indices...})`

Create a subplot spanning multiple cells (Julia Plots-style).

```cpp
figure(900, 600);

// Span cells 1 and 2 (top row)
subplot(2, 2, {1, 2});
plot(x, y);
title("Spans top row");

// Individual cells
subplot(2, 2, 3);
hist(data1);

subplot(2, 2, 4);
bar(x, values);
```

**Parameters:**
- `nrows`, `ncols`: Grid dimensions
- `{indices...}`: Initializer list of cell indices to span (1-based)

### `subplot_at(row, col)`

Position a subplot using 0-based row and column indices.

```cpp
figure(800, 600);
layout(2, 3);  // Set up 2x3 grid

subplot_at(0, 0);  // Row 0, Column 0 (top-left)
plot(x, y);

subplot_at(0, 2);  // Row 0, Column 2 (top-right)
scatter(x, y);

subplot_at(1, 1);  // Row 1, Column 1 (bottom-middle)
hist(data);
```

### `subplot_span(row1, col1, row2, col2)`

Create a subplot spanning from (row1, col1) to (row2, col2), using 0-based indices.

```cpp
figure(900, 700);
layout(3, 3);

// Large plot in top-left 2x2 area
subplot_span(0, 0, 1, 1);
plot(x, y);
title("2x2 area");

// Right column
subplot_span(0, 2, 0, 2);  // Single cell
subplot_span(1, 2, 1, 2);

// Bottom row
subplot_span(2, 0, 2, 2);  // Span 3 columns
hist(data);
```

## Layout Configuration

### `layout(nrows, ncols)`

Set up a basic grid layout.

```cpp
figure(800, 600);
layout(2, 3);  // 2 rows, 3 columns
```

### `layout(nrows, ncols, widthRatios)`

Set up a grid with custom column widths.

```cpp
figure(900, 600);
layout(2, 3, {1, 2, 1});  // Middle column is twice as wide

subplot(2, 3, 1);  // Small
subplot(2, 3, 2);  // Wide (2x)
subplot(2, 3, 3);  // Small
```

### `layout(nrows, ncols, widthRatios, heightRatios)`

Set up a grid with custom column widths and row heights.

```cpp
figure(800, 700);
layout(2, 2, {2, 1}, {1, 2});  // Left column wider, bottom row taller

subplot(2, 2, 1);  // Wide, short (top-left)
subplot(2, 2, 2);  // Narrow, short (top-right)
subplot(2, 2, 3);  // Wide, tall (bottom-left)
subplot(2, 2, 4);  // Narrow, tall (bottom-right)
```

## GridSpec Class

For full control over layout, use the `GridSpec` class.

### Constructor

```cpp
GridSpec gs(nrows, ncols);
```

### Methods

#### `setWidthRatios(ratios)`

Set relative widths for each column.

```cpp
GridSpec gs(2, 4);
gs.setWidthRatios({1, 2, 2, 1});  // Middle columns twice as wide
```

#### `setHeightRatios(ratios)`

Set relative heights for each row.

```cpp
GridSpec gs(3, 2);
gs.setHeightRatios({1, 2, 1});  // Middle row twice as tall
```

#### `setSpacing(wspace, hspace)`

Set horizontal and vertical spacing between subplots.

```cpp
gs.setSpacing(0.1, 0.15);  // 10% horizontal, 15% vertical
```

#### `setMargins(left, right, bottom, top)`

Set figure margins.

```cpp
gs.setMargins(0.1, 0.95, 0.08, 0.92);
```

### Complete Example

```cpp
figure(1000, 800);

GridSpec gs(3, 3);
gs.setWidthRatios({1, 2, 1});
gs.setHeightRatios({1, 2, 1});
gs.setSpacing(0.1, 0.15);
gs.setMargins(0.08, 0.95, 0.08, 0.92);
gcf().setLayout(gs);

// Now use subplot functions as normal
subplot(3, 3, {1, 2, 3});  // Top row
subplot(3, 3, 5);          // Middle-center (large)
// etc.
```

## Free Positioning

### `add_axes(left, bottom, width, height)`

Add an axes at an arbitrary position using normalized coordinates (0-1).

```cpp
figure(1000, 700);

// Main plot: left half
add_axes(0.1, 0.1, 0.5, 0.8);
plot(x, y);

// Side plot: right third
add_axes(0.65, 0.1, 0.3, 0.8);
hist(data);
```

**Parameters:**
- `left`: Left edge position (0 = left, 1 = right)
- `bottom`: Bottom edge position (0 = bottom, 1 = top)
- `width`: Width of axes (0-1)
- `height`: Height of axes (0-1)

### `inset_axes(x, y, width, height)`

Add an inset axes (plot within a plot).

```cpp
figure(800, 600);

// Main plot
subplot(1, 1, 1);
plot(x, y);
title("Main Plot");

// Inset showing zoomed detail
inset_axes(0.6, 0.6, 0.35, 0.3);
plot(x_zoom, y_zoom);
title("Zoomed");
```

## Helper Functions

### `gcf()`

Get current figure.

```cpp
Figure& fig = gcf();
fig.setLayout(gs);
```

### `gca()`

Get current axes.

```cpp
Axes& ax = gca();
ax.set_title("My Plot");
```

## Layout Examples

### Dashboard Layout

```cpp
figure(1200, 800);

// Top row: 4 KPI cards
GridSpec gs(3, 4);
gs.setHeightRatios({1, 2, 1});
gcf().setLayout(gs);

// KPI row
for (int i = 1; i <= 4; ++i) {
    subplot(3, 4, i);
    plot(kpi_x, kpi_y[i-1]);
    title("KPI " + std::to_string(i));
}

// Main chart (spans middle row)
subplot(3, 4, {5, 6, 7, 8});
plot(main_x, main_y);
title("Main Chart");

// Bottom row: detail charts
for (int i = 9; i <= 12; ++i) {
    subplot(3, 4, i);
    bar(categories, values[i-9]);
}
```

### Scientific Figure

```cpp
figure(1000, 800);
layout(2, 3, {2, 2, 1}, {2, 1});

// Large main plot
subplot_span(0, 0, 0, 1);  // Top-left 2 columns
plot(x, y);
title("Main Results");

// Inset with zoom
inset_axes(0.6, 0.15, 0.35, 0.25);
plot(x_detail, y_detail);

// Side panel
subplot_span(0, 2, 1, 2);  // Right column
hist(residuals);
title("Residuals");

// Bottom row
subplot(2, 3, 4);
scatter(pred, actual);
title("Predicted vs Actual");

subplot(2, 3, 5);
plot(time, error);
title("Error over time");
```

## Tips & Best Practices

1. **Use `layout()` first** - Set up the grid before creating subplots
2. **Consistent indexing** - `subplot()` uses 1-based, `subplot_at/span()` uses 0-based
3. **Width/height ratios** - Values are relative, {1,2,1} is same as {2,4,2}
4. **Margins and spacing** - Start with defaults, adjust if plots overlap
5. **GridSpec for complex layouts** - When you need full control
6. **add_axes for free-form** - When grid layout doesn't work
7. **inset_axes for details** - Great for zoomed views or supplementary info
